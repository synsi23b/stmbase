#pragma once

#include "rf24.h"

#define RF24_NODE_MAILBOX_COUNT 8
#define RF24_NODE_PAYLOAD_SIZE 27

class RF24Node
{
public:
  enum Protocol
  {
    request_address = 0x00,
    request_time = 0x01,
    log_key_value = 0x40
  };

  enum LinkMode
  {
    // lower 5 bits contain payload count
    // try to send only once
    unreliable = 0x20,
  };

  class Message
  {
    friend class RF24Node;

  public:
    void init(const char *receiver_address, RF24Node::Protocol protocol);

    void log(const char *key, uint8_t value);

    void send(RF24Node::LinkMode mode = unreliable);

  private:
    char *_log_key(const char *key);

    RF24Node::LinkMode _get_mode() const;
    uint8_t _get_payload_count() const;

    // hold various data about the package, not send
    uint8_t _mode;
    // hold receiver address, gets set by the node
    // to it's own address before sending
    char _address[4];
    RF24Node::Protocol _protocol;
    // adjust size by one for trailing zero
    char _payload[RF24_NODE_PAYLOAD_SIZE + 1];
  };

  typedef syn::MailBox<Message, RF24_NODE_MAILBOX_COUNT> Mailbox_t;

  // reserve a Message to fill with data
  // call message send method to try and send it
  // returns 0 if the message pool is full already
  static Message *try_reserve();

  static void message_handler_routine(const char *this_node_address);

private:
  static Mailbox_t _inbox;
  //static const char *_this_node_address;
  static RF24 _radio;
};