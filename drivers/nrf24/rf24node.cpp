#include "rf24node.h"
//#include <stdio.h>
//#include <string.h>

RF24Node::Mailbox_t RF24Node::_outbox;
//const char *RF24Node::_this_node_address;
RF24 RF24Node::_radio;

uint8_t RF24Node::_inbuffer[32];

// count the amount of successfull and failed messages
uint16_t RF24Node::_success_counter = 0;
uint16_t RF24Node::_failure_counter = 0;
uint16_t RF24Node::_outbox_overflow_counter = 0;

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
  _mode = res - (char *)_payload;
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
  // address + protocoll + count bytes of mode
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

void RF24Node::message_handler_routine(uint16_t pconfig)
{
  uint8_t adrbuffer[4];
  uint16_t this_node_address = ((Config *)pconfig)->this_node_address;
  void (*indata_handler)(uint8_t *) = ((Config *)pconfig)->indata_handler;
  Message::unfold_address(this_node_address, adrbuffer);
  bool init_success = _radio.init((const char *)adrbuffer);

  if (indata_handler != 0)
  {
    _radio.listen();
  }

#ifdef DEBUG
  while(!init_success)
    ;
#endif

  Message *mail_to_send = 0;
  uint8_t retry_count = 0;
  uint8_t packet_id_counter = 0;
  uint16_t sleep_counter = 0;
  while (true != false)
  {
    if (indata_handler != 0)
    {
    }
    else
    {
      syn::sleep(137);
    }
    ++sleep_counter;
    if (mail_to_send != 0)
    {
      if (retry_count > 0)
      {
        retry_count -= 8;
        if (retry_count == 0)
        {
          // our retries are over, purge the message
          _outbox.purge();  // release message back to pool
          mail_to_send = 0; // make sure to grab a new message
          continue;
        }
      }
      if (_radio.write((uint8_t *)(&mail_to_send->_address), mail_to_send->_get_payload_count()))
      {
        _outbox.purge();  // release message back to pool
        mail_to_send = 0; // make sure to grab a new message
        ++_success_counter;
      }
      else
      {
        ++_failure_counter;
      }
    }
    else if (_outbox.try_peek(&mail_to_send))
    {
      // setup message addresses
      Message::unfold_address(mail_to_send->_address, adrbuffer);
      _radio.set_destination((const char *)adrbuffer);
      //memcpy(mail_to_send->_address, this_node_address, 4);
      mail_to_send->_address = this_node_address;
      mail_to_send->_set_packet_id(packet_id_counter);
      // 8 bit packet id counter rolls over to 0 packet id in 4 rounds
      packet_id_counter += 0x40;
      // try to send the message
      bool success = _radio.write((uint8_t *)(&mail_to_send->_address), mail_to_send->_get_payload_count());
      if (!success)
      {
        ++_failure_counter;
        // if we weren't successfull, check the message mode to determine the next action
        LinkMode mode = mail_to_send->_get_mode();
        if (mode != unreliable)
        {
          retry_count = ((uint8_t)mode) & 0xE0;
          continue;
        }
      }
      else
      {
        if ((mail_to_send->_protocol & RF24_NODE_PROTOCOLL_MASK) == radio_state)
        {
          // success in sending our radio state, reset the counters
          _success_counter = 0;
          _failure_counter = 0;
          _outbox_overflow_counter = 0;
        }
        else
        {
          ++_success_counter;
        }        
      }
      // successfull send or unreliable message gets here
      // also get here if we reached the end of a retry counter
      _outbox.purge();  // release message back to pool
      mail_to_send = 0; // make sure to grab a new message
    }
    else
    {
      // no mail to send, try to send radio status message if the time has come for that
      // 20 minutes / 137 millliseconds ~ 8759 iterations
      if (sleep_counter > 8759)
      {
        mail_to_send = try_reserve(RF24_NODE_LOGGER, radio_state);
        if (mail_to_send)
        {
          //mail_to_send->_radio_state((uint16_t *)&_success_counter);
          mail_to_send->log_u16("s", _success_counter);
          mail_to_send->log_u16("f", _failure_counter);
          mail_to_send->log_u16("o", _outbox_overflow_counter);
          mail_to_send->send();
          mail_to_send = 0;
          sleep_counter = 0;
        }
      }
    }
  }
}