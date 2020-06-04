#include "synhal.h"
#if (SYN_ENABLE_EEPROM == 1)
using namespace syn;

VirtualEeprom::Flash VirtualEeprom::Flash::_instance;

VirtualEeprom::Flash::Flash()
{
  for (uint16_t i = 0; i < VE_BANK_COUNT; ++i)
    _banks[i] = 0;
  // unlock the flash memory controller
  if (FLASH->CR & FLASH_CR_LOCK)
  {
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
  }
}

// write to specific address and check for success
bool VirtualEeprom::Flash::write(uint16_t *address, uint16_t value, bool use_lock)
{
  if (use_lock)
  {
    while (true)
    {
      OS_EnterRegion();
      if (!(FLASH->SR & FLASH_SR_BSY))
        break;
      OS_LeaveRegion();
    }
  }
  else
  {
    while (FLASH->SR & FLASH_SR_BSY)
      ;
  }
  FLASH->CR = FLASH_CR_PG;
  *address = value;
  if (use_lock)
    OS_LeaveRegion();
  while (FLASH->SR & FLASH_SR_BSY)
    ;
  return *address == value;
}

// erase the entire page
void VirtualEeprom::Flash::erasePage(uint16_t *pageaddress, bool use_lock)
{
  if (use_lock)
  {
    while (true)
    {
      OS_EnterRegion();
      if (!(FLASH->SR & FLASH_SR_BSY))
        break;
      OS_LeaveRegion();
    }
  }
  else
  {
    while (FLASH->SR & FLASH_SR_BSY)
      ;
  }
  FLASH->CR = FLASH_CR_PER;
  FLASH->AR = (uint32_t)pageaddress;
  FLASH->CR |= FLASH_CR_STRT;
  if (use_lock)
    OS_LeaveRegion();
}

// protected bank creator
VirtualEeprom::Bank *VirtualEeprom::Flash::getBank(uint16_t number)
{
  Bank *pb = 0;
  Atomic a;
  if (_instance._banks[number] == 0)
  {
    pb = new Bank(number);
    _instance._banks[number] = pb;
  }
  else
  {
    pb = _instance._banks[number];
  }
  return pb;
}

VirtualEeprom::Bank::Entry *VirtualEeprom::Bank::Entry::getPrev()
{
  uint16_t *pt = (uint16_t *)this;
  pt -= loc_prev;
  return (Entry *)pt;
}

VirtualEeprom::Bank::Entry *VirtualEeprom::Bank::Entry::getNext()
{
  const uint16_t *pt = &data;
  pt += ((size_this + 1) / 2);
  return (Entry *)pt;
}

bool VirtualEeprom::Bank::Entry::write(Entry *previous, uint16_t vaddress, const uint8_t *value, uint16_t size, bool use_lock)
{
  bool success = true;
  OS_ASSERT(this->vaddress == 0xFFFF, ERR_NULL_POINTER); // is actually deleted
  OS_ASSERT(size <= 0xFF, ERR_BAD_INDEX);
  uint16_t locpre_size = (uint16_t *)this - (uint16_t *)previous;
  // we have to write the entire halfword containing size and loc prev in one go.
  // this is little endian, so loc_pre is actually the low byte and size the high byte
  locpre_size |= (size << 8);
  if (!Flash::write((uint16_t *)&loc_prev, locpre_size, use_lock))
    success = false;
  uint16_t *rp = (uint16_t *)value;
  uint16_t *wp = (uint16_t *)&data;
  uint16_t *ep = rp + ((size + 1) / 2);
  for (; rp < ep; ++rp, ++wp)
    if (!Flash::write(wp, *rp, use_lock))
      success = false;
  // write vaddress last in order to recover from powerloss and not corrupt this variable
  if (!Flash::write((uint16_t *)&this->vaddress, vaddress, use_lock))
    success = false;
  return success;
}

void VirtualEeprom::Bank::Entry::read(uint8_t *value, uint16_t size)
{
  std::memcpy(value, &data, size);
}

#define VIRTEEPROM_ERASED 0x0000
#define VIRTEEPROM_ERASED_INDEX 0
#define VIRTEEPROM_ENTRY_START 0x0400 // dummy information for fake entry at start of the table
#define VIRTEEPROM_ENTRY_START_INDEX 1
#define VIRTEEPROM_RECEIVE 0xADDE
#define VIRTEEPROM_RECEIVE_INDEX 2
#define VIRTEEPROM_VALID 0xDEC0
#define VIRTEEPROM_VALID_INDEX 3
#define VIRTEEPROM_DATA_START_INDEX 4
#define VIRTEEPROM_DATA_END_INDEX (VirtualEeprom::VE_PAGE_SIZE / 2)

VirtualEeprom::Bank::Bank(uint16_t number)
{
  _mutex.init();
  uint16_t page = VE_BANK_HIGHEST - number * 2; // bank 0 is page 62 and 63
  _pages[0] = (uint16_t *)(VE_FLASH_BASE + page * VE_PAGE_SIZE);
  _pages[1] = (uint16_t *)(VE_FLASH_BASE + (page + 1) * VE_PAGE_SIZE);

  _active = 0;
  // check in reverse order for possible states to find the valid page
  if (_pages[0][VIRTEEPROM_VALID_INDEX] == VIRTEEPROM_VALID)
  {
    // powerloss during copy
    if (_pages[1][VIRTEEPROM_RECEIVE_INDEX] == VIRTEEPROM_RECEIVE)
    {
      copy(_pages[0], _pages[1], false);
    }
    // all good! just find write pointer
    else if (_pages[1][VIRTEEPROM_ERASED_INDEX] == VIRTEEPROM_ERASED)
    {
      _active = _pages[0];
    }
  }
  else if (_pages[0][VIRTEEPROM_RECEIVE_INDEX] == VIRTEEPROM_RECEIVE)
  {
    // powerloss during copy
    if (_pages[1][VIRTEEPROM_VALID_INDEX] == VIRTEEPROM_VALID)
    {
      copy(_pages[1], _pages[0], false);
    }
    // powerloss at the end of copy from 1 to 0, mark 0 as active
    else if (_pages[1][VIRTEEPROM_ERASED_INDEX] == VIRTEEPROM_ERASED)
    {
      _active = _pages[0];
      Flash::write(&(_pages[0][VIRTEEPROM_VALID_INDEX]), VIRTEEPROM_VALID, false);
    }
  }
  else if (_pages[0][VIRTEEPROM_ERASED_INDEX] == VIRTEEPROM_ERASED)
  {
    // all good, just find write pointer
    if (_pages[1][VIRTEEPROM_VALID_INDEX] == VIRTEEPROM_VALID)
    {
      _active = _pages[1];
    }
    // powerloss at the end of copy from 0 to 1, mark 1 as active
    else if (_pages[1][VIRTEEPROM_RECEIVE_INDEX] == VIRTEEPROM_RECEIVE)
    {
      _active = _pages[1];
      Flash::write(&(_pages[1][VIRTEEPROM_VALID_INDEX]), VIRTEEPROM_VALID, false);
    }
  }
  // no valid header on page 0
  if (_active == 0)
  {
    format();
  }
  updateWritePointer();
}

bool VirtualEeprom::Bank::read(uint16_t vaddress, uint8_t *value, uint16_t size)
{
  bool success = false;
  _mutex.lock();
  Entry *vptr = _wpointer;    // the current writepointer still points at the newest entry
  while (vptr->vaddress != 0) // dummy entry at start of the table has this address
  {
    if (vptr->vaddress == vaddress)
    { // found the variable we are looking for. if the size matches we are a winner
      // if not, return error and dont read it
      if (vptr->size_this == size)
      {
        vptr->read(value, size);
        success = true;
      }
      break;
    }
    vptr = vptr->getPrev();
  }
  _mutex.unlock();
  return success;
}

bool VirtualEeprom::Bank::write(uint16_t vaddress, const uint8_t *value, uint16_t size)
{
  OS_ASSERT(vaddress != 0xFFFF && vaddress != 0, ERR_NULL_POINTER);
  OS_ASSERT(size <= 0xFF, ERR_BAD_INDEX);
  bool ret = false;
  bool full = false;
  LockGuard<Mutex> lg(_mutex);
  Entry *pnext = _wpointer->getNext(); // the current writepointer still points at the newest entry
  // check if we got enough space left for the new variable
  uint16_t remaining = &_active[VIRTEEPROM_DATA_END_INDEX] - (uint16_t *)pnext;
  if (remaining < ((size + 5) / 2))
  { // not enough space, copy the newest values over to the alternate page
    if (_active == _pages[0])
      copy(_pages[0], _pages[1]);
    else
      copy(_pages[1], _pages[0]);
    pnext = _wpointer->getNext();
    full = (&_active[VIRTEEPROM_DATA_END_INDEX] - (uint16_t *)pnext) < ((size + 5) / 2);
  }
  if (!full)
  { // write the new value into the rom, if succeeds, update w_pointer
    ret = pnext->write(_wpointer, vaddress, value, size, true);
    if (ret)
      _wpointer = pnext;
  }
  return ret;
}

void VirtualEeprom::Bank::initPage(uint16_t *pageaddress, bool use_lock)
{
  Flash::erasePage(pageaddress, use_lock);
  Flash::write(&(pageaddress[VIRTEEPROM_ENTRY_START_INDEX]), VIRTEEPROM_ENTRY_START, use_lock);
  Flash::write(&(pageaddress[VIRTEEPROM_ERASED_INDEX]), VIRTEEPROM_ERASED, use_lock);
}

void VirtualEeprom::Bank::format()
{
  // format should only ever be called in the initialozation of a Bank, therefore no locks
  initPage(_pages[0], false);
  initPage(_pages[1], false);
  Flash::write(&(_pages[0][VIRTEEPROM_VALID_INDEX]), VIRTEEPROM_VALID, false);
  _active = _pages[0];
}

void VirtualEeprom::Bank::copy(uint16_t *src, uint16_t *dst, bool use_lock)
{
  // set the write pointer to the start of the new page
  Entry *wpprev = (Entry *)dst;
  Entry *writeptr = wpprev->getNext();
  // check if we were in the middle of a receive or not
  if (dst[VIRTEEPROM_RECEIVE_INDEX] == VIRTEEPROM_RECEIVE)
  {
    while (writeptr->vaddress != 0xFFFF)
    {
      wpprev = writeptr;
      writeptr = writeptr->getNext();
    }
  }
  else
  {
    // start copy by setting the correct status on receiving page
    Flash::write(&dst[VIRTEEPROM_RECEIVE_INDEX], VIRTEEPROM_RECEIVE, use_lock);
  }
  // set the read pointer to the newest virtual address
  Entry *rpprev = (Entry *)src;
  Entry *rpread = rpprev->getNext();
  Entry *rpend = (Entry *)&src[VIRTEEPROM_DATA_END_INDEX];
  while (rpread->vaddress != 0xFFFF && rpread < rpend)
  {
    rpprev = rpread;
    rpread = rpread->getNext();
  }
  rpread = rpprev;
  // start copying unique values
  while (rpread->vaddress != 0) // going backwards through our source page. dummy entry has address 0
  {
    // check if this vaddress was copied already
    Entry *searchptr = (Entry *)(&dst[VIRTEEPROM_DATA_START_INDEX]);
    while (searchptr != writeptr)
    {
      if (searchptr->vaddress == rpread->vaddress)
        break;
      searchptr = searchptr->getNext();
    }
    if (searchptr == writeptr)
    {
      // was not found, copy the vadr data pair
      writeptr->write(wpprev, rpread->vaddress, (const uint8_t *)&rpread->data, rpread->size_this, use_lock);
      wpprev = writeptr;
      writeptr = writeptr->getNext();
    }
    rpread = rpread->getPrev();
  }
  // done copying, erase src page
  initPage(src, use_lock);
  // mark dst as new valid page
  Flash::write(&dst[VIRTEEPROM_VALID_INDEX], VIRTEEPROM_VALID, use_lock);
  _active = dst;
  _wpointer = wpprev;
}

void VirtualEeprom::Bank::updateWritePointer()
{
  Entry *pprev = (Entry *)_active;
  Entry *pcur = (Entry *)(_active + VIRTEEPROM_DATA_START_INDEX);
  Entry *pend = (Entry *)(_active + VIRTEEPROM_DATA_END_INDEX);
  while (pcur < pend)
  {
    if (pcur->vaddress == 0xFFFF)
      break;
    pprev = pcur;
    pcur = pcur->getNext();
  }
  _wpointer = pprev;
}
#endif // #if (SYN_ENABLE_EEPROM == 1)
