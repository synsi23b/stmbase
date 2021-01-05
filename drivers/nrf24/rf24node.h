#pragma once

#include "rf24.h"

#define RF24_NODE_MAILBOX_COUNT 8
#define RF24_NODE_PAYLOAD_SIZE 29

// put in 2 chars (A..Z) and a number (0..63), get a folded 4 byte address
#define RF24_NODE_FOLD_ADDR(a, b, x) (((a - 'A') << 11) | ((b - 'A') << 6) | (x))

#define RF24_NODE_LOGGER RF24_NODE_FOLD_ADDR('L', 'G', 1)

class RF24Node
{
public:
#define RF24_NODE_PROTOCOLL_MASK 0x3F
  enum Protocol
  {
    request_address = 0x00,
    request_time = 0x01,
    radio_state = 0x20,
    log_key_value = 0x21,
    message_info = 0x22,
    message_warning = 0x23,
    message_error = 0x24,
    task_started = 0x25,
    task_progress = 0x26,
    task_completed = 0x27,
    task_canceled = 0x28,
    temperature_value = 0x29,
    // 0x3F is maximum!
    // upper 2 bits are packet id to detect multiple received
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
    // These functions are for the log_key_value protocol
    void log_u8(const char *key, uint8_t value);
    void log_i8(const char *key, int8_t value);
    void log_u16(const char *key, uint16_t value);
    void log_i16(const char *key, int16_t value);
    // void log_u32(const char *key, uint32_t value);
    // void log_i32(const char *key, int32_t value);
    // Log 0 terminated strings.
    // The strings shouldn't start with a - or a number, as they will be than identified as integers
    // The character \3 (ETX End of Text) is used as a termination character and therefore forbidden
    // however there is no check of usage of this character, so do whatever you want, lol!
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

    // send the completed message to the receiver
    // linkmode can select amount of attempts to send the message again
    // or send it forever, which would be the only mode to be called "reliable"
    // but it can choke the Node message buffer, if the destination is not reachable
    void send(RF24Node::LinkMode mode = unreliable);

    // buffer needs to be 4 byte
    static void unfold_address(uint16_t address, uint8_t *buffer);

  private:
    void _init(uint16_t folded_receiver_address, RF24Node::Protocol protocol);
    char *_log_key(const char *key);

    RF24Node::LinkMode _get_mode() const;
    uint8_t _get_payload_count() const;
    void _set_packet_id(uint8_t id);

    // hold various data about the package, not send
    uint8_t _mode;
    // hold receiver address, gets set by the node
    // to it's own address before sending
    uint16_t _address;
    RF24Node::Protocol _protocol;
    // adjust size by one for trailing zero
    uint8_t _payload[RF24_NODE_PAYLOAD_SIZE + 1];
  };

  typedef syn::MailBox<Message, RF24_NODE_MAILBOX_COUNT> Mailbox_t;

  // reserve a Message to fill with data
  // call message send method to try and send it
  // returns 0 if the message pool is full already
  static Message *try_reserve(uint16_t folded_receiver_address, Protocol protocol);

  static void message_handler_routine(uint16_t this_node_address);

private:
  static Mailbox_t _inbox;
  //static const char *_this_node_address;
  static RF24 _radio;
  // addresses past this line are included in the radio state message
  // currently that is only the success and failure counter
  static uint16_t _success_counter;
  static uint16_t _failure_counter;
};