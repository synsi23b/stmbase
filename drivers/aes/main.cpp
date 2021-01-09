#include "aes.hpp"
#include <cstring>

const char* sdata = "Hello C+-+ World from VS Code and the C+-+ extension! You FUcker";
const char* key = "whateverWHATEVER";
const char* iv = "WHATEVERwhatever";

AES_ctx ctx;

uint8_t buffer[32];

void AES_ECB_encrypt_buffer(AES_ctx* ctx, uint8_t* buffer, uint8_t length)
{
  uint8_t *end = buffer + length;
  for(;buffer < end; buffer += 16)
    AES_ECB_encrypt(ctx, buffer);
}

void AES_ECB_decrypt_buffer(AES_ctx* ctx, uint8_t* buffer, uint8_t length)
{
  uint8_t *end = buffer + length;
  for(;buffer < end; buffer += 16)
    AES_ECB_decrypt(ctx, buffer);
}

int main()
{
    memcpy(buffer, sdata, 32);

    AES_init_ctx(&ctx, (uint8_t*)key);

    AES_ECB_encrypt_buffer(&ctx, buffer, 32);
    
    AES_ECB_decrypt_buffer(&ctx, buffer, 32);
}
