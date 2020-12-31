#include "rf24node.h"
#include <stdio.h>
#include <string.h>

RF24Node::Mailbox_t RF24Node::_inbox;
//const char *RF24Node::_this_node_address;
RF24 RF24Node::_radio;

void RF24Node::Message::init(const char *receiver_address, RF24Node::Protocol protocol)
{
  memcpy(_address, receiver_address, 4);
  _protocol = protocol;
  _mode = 0;
}

void RF24Node::Message::log(const char *key, uint8_t value)
{
  char *res = _log_key(key);
  int num = sprintf(res, "%u,", value);
#ifdef DEBUG
  while (num < 0)
    ;
#endif
  _mode += num;
}

char *RF24Node::Message::_log_key(const char *key)
{
  //uint8_t remaining = RF24_NODE_PAYLOAD_SIZE - _mode;
  // char *res = strcpy(_payload + _mode, key);
  // *res = ':';
  // _mode = res - _payload;
  int num = sprintf(_payload + _mode, "%s:", key);
#ifdef DEBUG
  while (num < 0)
    ;
#endif
  _mode += num;
  return _payload + _mode;
}

RF24Node::LinkMode RF24Node::Message::_get_mode() const
{
  return (LinkMode)(_mode & 0xE0);
}

uint8_t RF24Node::Message::_get_payload_count() const
{
  // address + protocoll + count bytes of mode
  return (_mode & 0x1F) + 5;
}

void RF24Node::Message::send(RF24Node::LinkMode mode)
{
  // set mode on how to handle message
  _mode |= mode;
  // release message to inbox, will be send by msg handler
  RF24Node::_inbox.release();
}

RF24Node::Message *RF24Node::try_reserve()
{
  Message *pres = 0;
  _inbox.try_reserve(&pres);
  return pres;
}

void RF24Node::message_handler_routine(const char *this_node_address)
{
  _radio.init(this_node_address);

  Message *mail_to_send = 0;
  while (true != false)
  {
    if (_inbox.try_peek(&mail_to_send))
    {
      _radio.set_destination(mail_to_send->_address);

      LinkMode mode = mail_to_send->_get_mode();

      if (mode == unreliable)
      {
        memcpy(mail_to_send->_address, this_node_address, 4);
        _radio.write((uint8_t *)mail_to_send->_address, mail_to_send->_get_payload_count());
        _inbox.purge(); // release message back to pool
      }
      else
      {
        while (true)
          ;
      }
    }
    syn::sleep(137);
  }
}