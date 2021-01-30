#include "rf24node.h"
//#include <stdio.h>
//#include <string.h>

#if RF24_NODE_MAILBOX_COUNT != 0
RF24Node::Mailbox_t RF24Node::_outbox;
RF24Node::Message *RF24Node::_current_msg;
uint16_t RF24Node::_this_node_address;

uint8_t RF24Node::_inbuffer[32];

// count the amount of successful and failed messages
uint16_t RF24Node::_success_counter = 0;         // count ACK messages
uint16_t RF24Node::_failure_counter = 0;         // count NACK messages
uint16_t RF24Node::_lost_message_counter = 0;    // count NACK & purged messages
uint16_t RF24Node::_outbox_overflow_counter = 0; // count times message reservation failed



#ifdef STM8S
template <typename T>
void _store_big_endian_16(uint8_t *dest, T value)
{
  *((T *)dest) = value;
}
#else
template <typename T>
void _store_big_endian_16(uint8_t *dest, T value)
{
  *dest++ = (value >> 8) & 0xFF;
  *dest = value & 0xFF;
}
#endif

void RF24Node::Message::_init(uint16_t folded_receiver_address, RF24Node::Protocol protocol)
{
  _address = folded_receiver_address;
  _protocol = protocol;
  _mode = 0;
}

void RF24Node::Message::log_u8(const char *key, uint8_t value)
{
  uint8_t *res = (uint8_t *)_log_key(key, '\1');
  *res = value;
  _mode = (res - _payload) + 1;
}

void RF24Node::Message::log_i8(const char *key, int8_t value)
{
  uint8_t *res = (uint8_t *)_log_key(key, '\2');
  *res = value;
  _mode = (res - _payload) + 1;
}

void RF24Node::Message::log_u16(const char *key, uint16_t value)
{
  uint8_t *res = (uint8_t *)_log_key(key, '\3');
  _store_big_endian_16(res, value);
  _mode = (res - _payload) + 2;
#ifdef DEBUG
  while (_mode > RF24_NODE_PAYLOAD_SIZE)
    ;
#endif
}

void RF24Node::Message::log_i16(const char *key, int16_t value)
{
  uint8_t *res = (uint8_t *)_log_key(key, '\4');
  _store_big_endian_16(res, value);
  _mode = (res - _payload) + 2;
#ifdef DEBUG
  while (_mode > RF24_NODE_PAYLOAD_SIZE)
    ;
#endif
}

void RF24Node::Message::log_str(const char *key, const char *string)
{
  char *res = _log_key(key, '\5');
  res = syn::Utility::strcpy(res, string);
  *res++ = '\0';
  _mode = (res - (char *)_payload) + 1;
#ifdef DEBUG
  while (_mode > RF24_NODE_PAYLOAD_SIZE)
    ;
#endif
}

char *RF24Node::Message::_log_key(const char *key, char seperator)
{
#ifdef DEBUG
  Protocol proto = _protocol;
  while (proto != log_key_value && proto != radio_state)
    ;
#endif
  char *writep = (char *)_payload + _mode;
  char *resp = syn::Utility::strcpy(writep, key);
  *resp++ = seperator;
  return resp;
}

void RF24Node::Message::message(const char *message)
{
#ifdef DEBUG
  Protocol proto = _protocol;
  while (proto != message_info && proto != message_warning && proto != message_error)
    ;
#endif
  char *res = syn::Utility::strcpy((char *)_payload, message);
  _mode = res - (char *)_payload;
#ifdef DEBUG
  while (_mode > RF24_NODE_PAYLOAD_SIZE)
    ;
#endif
}

void RF24Node::Message::task(const char *description, uint16_t remaining_seconds, uint8_t progress_percent)
{
#ifdef DEBUG
  Protocol proto = _protocol;
  while (proto != task_started && proto != task_progress && proto != task_completed && proto != task_canceled)
    ;
#endif
  _payload[0] = progress_percent;
  _store_big_endian_16(_payload + 1, remaining_seconds);
  char *res = syn::Utility::strcpy((char *)_payload + 3, description);
  _mode = res - (char *)_payload;
#ifdef DEBUG
  while (_mode > RF24_NODE_PAYLOAD_SIZE)
    ;
#endif
}

void RF24Node::Message::temperature(const uint8_t *thermometer_id, int16_t value)
{
#ifdef DEBUG
  while (_protocol != temperature_value)
    ;
#endif
  uint8_t *buff = _payload;
  const uint8_t *rpoint = thermometer_id;
  for (uint8_t i = 0; i < 8; ++i)
  {
    *buff++ = *rpoint++;
  }
  _store_big_endian_16(&_payload[8], value);
  _mode = 10;
}

RF24Node::LinkMode RF24Node::Message::_get_mode() const
{
  return (LinkMode)(_mode & 0xE0);
}

uint8_t RF24Node::Message::_get_payload_count() const
{
  // address + protocol + count bytes of mode
  return (_mode & 0x1F) + 3;
}

void RF24Node::Message::_set_packet_id(uint8_t id)
{
  _protocol = (Protocol)(_protocol | id);
}

void RF24Node::Message::send(RF24Node::LinkMode mode)
{
  // set mode on how to handle message
  _mode |= mode;
  // release message to inbox, will be send by msg handler
  RF24Node::_outbox.release();
}

void RF24Node::Message::unfold_address(uint16_t address, uint8_t *buffer)
{
  buffer += 3;
  uint8_t tmp = address & 0x3F;
  *buffer-- = (tmp % 10) + '0';
  *buffer-- = (tmp / 10) + '0';
  address >>= 6;
  tmp = (address & 0x1F) + 'A';
  *buffer-- = tmp;
  address >>= 5;
  tmp = address + 'A';
  *buffer = tmp;
}

RF24Node::Message *RF24Node::try_reserve(uint16_t folded_receiver_address, Protocol protocol)
{
  Message *pres = 0;
  _outbox.try_reserve(&pres);
  if (pres)
  {
    pres->_init(folded_receiver_address, protocol);
  }
  else
  {
    ++_outbox_overflow_counter;
  }
  return pres;
}

bool RF24Node::_try_setup_next_message()
{
  static uint8_t packet_id_counter = 0;
#ifdef DEBUG
  while (_current_msg != 0)
    ;
#endif
  if (_outbox.try_peek(&_current_msg))
  {
    // setup message addresses
    uint8_t adrbuffer[4];
    Message::unfold_address(_current_msg->_address, adrbuffer);
    RF24::set_destination((const char *)adrbuffer);
    _current_msg->_address = _this_node_address;
    _current_msg->_set_packet_id(packet_id_counter);
    // 8 bit packet id counter rolls over to 0 packet id in 4 rounds
    packet_id_counter += 0x40;
    return true;
  }
  return false;
}

bool RF24Node::_try_send()
{
  bool success = RF24::write((uint8_t *)(&_current_msg->_address), _current_msg->_get_payload_count());
  if (success)
  {
    ++_success_counter;
  }
  else
  {
    ++_failure_counter;
  }
  return success;
}

void RF24Node::_purge_current_message(bool success)
{
  if (!success)
  {
    ++_lost_message_counter;
  }
  _outbox.purge();
  _current_msg = 0;
}

void RF24Node::message_handler_routine(uint16_t pconfig)
{
  void (*indata_handler)(uint8_t *) = ((Config *)pconfig)->indata_handler;
  _this_node_address = ((Config *)pconfig)->this_node_address;

  {
    uint8_t adrbuffer[4];
    Message::unfold_address(_this_node_address, adrbuffer);
    while (true)
    {
      if (RF24::init((const char *)adrbuffer))
      {
        break;
      }
      syn::sleep(150);
    }
  }

  if (indata_handler != 0)
  {
    RF24::listen();
  }

  _current_msg = 0;
  uint8_t retry_count = 0;
  uint16_t sleep_counter = 0;
  while (true != false)
  {
    if (indata_handler != 0)
    {
      syn::sleep(137);
    }
    else
    {
      syn::sleep(137);
    }
    ++sleep_counter;

    // handle outgoing messages
    if (_current_msg != 0)
    {
      // current message is not null, retry to send it
      if (retry_count > 0)
      {
        retry_count -= 8;
        if (retry_count == 0)
        {
          // our retries are over, purge the message instead of sending
          _purge_current_message(false);
          continue; // skip everything, the retry message ran out
        }
      }
      if (_try_send())
      {
        _purge_current_message(true);
      }
    }
    else if (_try_setup_next_message())
    {
      // try to send the message
      bool success = _try_send();
      if (!success)
      {
        // if we weren't successful, check the message mode to determine the next action
        LinkMode mode = _current_msg->_get_mode();
        if (mode != unreliable)
        {
          retry_count = ((uint8_t)mode) & 0xE0;
          continue; // skip purging the message, as it is a retry one
        }
      }
      else
      {
        if ((_current_msg->_protocol & RF24_NODE_PROTOCOLL_MASK) == radio_state)
        {
          // success in sending our radio state, reset the counters
          sleep_counter = 0;
          _success_counter = 0;
          _failure_counter = 0;
          _lost_message_counter = 0;
          _outbox_overflow_counter = 0;
        }
      }
      // successful send or unreliable failure message gets here
      _purge_current_message(success);
    }
    else
    {
      // outbox is empty and _current_msg is null
      if (sleep_counter > 8759)
      {
        // 20 minutes / 137 milliseconds ~ 8759 iterations
        // periodically send the status of this node
        _current_msg = try_reserve(RF24_NODE_LOGGER, radio_state);
        if (_current_msg)
        {
          _current_msg->log_u16("s", _success_counter);
          _current_msg->log_u16("f", _failure_counter);
          _current_msg->log_u16("l", _lost_message_counter);
          _current_msg->log_u16("o", _outbox_overflow_counter);
          _current_msg->send(RF24Node::unreliable); // definitely send only once, since we don't reset counters in the retry-arc
          _current_msg = 0;
        }
      }
    }
  }
}

#endif // #if RF24_NODE_MAILBOX_COUNT != 0