#pragma once

#include "rf24.h"

#define RF24_NODE_MAILBOX_COUNT 8
#define RF24_NODE_PAYLOAD_SIZE 27

class RF24Node
{
public:
  enum Protocol
  {
    // upper 2 bits are packet id to detect multiple received
    request_address = 0x00,
    request_time = 0x01,
    radio_state = 0x1F,
    log_key_value = 0x20,
    message_info = 0x21,
    message_warning = 0x22,
    message_error = 0x23,
    task_started = 0x26,
    task_progress = 0x37,
    task_completed = 0x38,
    temperature_value = 0x3A,
  };

  enum LinkMode
  {
    // lower 5 bits contain payload count

    // if sending failed, wait a bit and retry
    // can seriously block this node from doing anything else
    // also might be just the node thinking it didn't suceed
    retry_forever = 0x00,
    // wait a bit between sending and try a total of 4 times
    retry_4 = 0x20,
    retry_8 = 0x40,
    retry_12 = 0x60,
    retry_16 = 0x80,
    retry_20 = 0xA0,
    retry_24 = 0xC0,
    // try to send only once (+ the auto retries of NRF24 Module)
    unreliable = 0xE0,
  };

  class Message
  {
    friend class RF24Node;

  public:
    void init(const char *receiver_address, RF24Node::Protocol protocol);

    // These functions are for the log_key_value protocol
    void log_u8(const char *key, uint8_t value);
    void log_i8(const char *key, int8_t value);
    void log_u16(const char *key, uint16_t value);
    void log_i16(const char *key, int16_t value);
    // Log 0 terminated strings.
    // The strings shouldn't start with a - or a number, as they will be than identified as integers
    // The character \6 (Acknowlege) is used as a termination character and therefore forbidden
    void log_str(const char *key, const char *string);

    // send strings with different kind of log level
    // the log level is determined by the protocoll
    // message_info, message_warning, message_error
    void message(const char *message);

    // send a status update about a started or running task
    // task state is defined by the protocol
    // task_started, task_progress, task_completed
    // the description string should be a maximum of 24 bytes long (excluding trailing zero)
    void task(const char *description, uint16_t remaining_seconds, uint8_t progress_percent);

    // send the temperature measured on one DS18B20 Thermometer
    // thermometer_id is a pointer to an 8 byte themometer ID array, wil be send as a hex string
    // followed directly by temperature value as little endian integer in 100th degrees celsius
    // i.e. if the value is 3556 -> 35.56 degrees
    void temperature(const uint8_t *thermometer_id, int16_t value);

    void send(RF24Node::LinkMode mode = unreliable);

  private:
    void _radio_state(uint16_t *pointer_to_int32x2);
    char *_log_key(const char *key);

    RF24Node::LinkMode _get_mode() const;
    uint8_t _get_payload_count() const;
    void _set_packet_id(uint8_t id);

    // hold various data about the package, not send
    uint8_t _mode;
    // hold receiver address, gets set by the node
    // to it's own address before sending
    char _address[4];
    RF24Node::Protocol _protocol;
    // adjust size by one for trailing zero
    uint8_t _payload[RF24_NODE_PAYLOAD_SIZE + 1];
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
  // addresses past this line are included in the radio state message
  // currently that is only the success and failure counter
  static uint32_t _success_counter;
  static uint32_t _failure_counter;
};