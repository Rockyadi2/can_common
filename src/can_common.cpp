#include <can_common.h>

// !!! FFS. Is it a "handler", a "filter", or a "callback" ...pick a naming convention and stick with it!

//+=====================================================================================================================
CANListener::CANListener ()
{
    numFilters = 8;
	initialize();
}

//+=====================================================================================================================
void  CANListener::initialize ()
{
	callbacksActive = 0x00;  // Bits 0..7 are the mailboxe callbacks;  bit 8 is the general callback
}

//+=====================================================================================================================
// !!! This needs to return a bool
void  CANListener::attachMBHandler (uint8_t mailbox)
{
	if (mailBox >= numFilters)  return ;//false;

	callbacksActive |= (1 << mailbox) ;
	return ;//true;
}

//+=====================================================================================================================
// !!! This needs to return a bool
void  CANListener::detachMBHandler (uint8_t mailbox)
{
	if (mailBox >= numFilters)  return ;//false;

	callbacksActive &= ~(1 << mailbox) ;
	return ;//true;
}

//+=====================================================================================================================
void  CANListener::attachGeneralHandler ()
{
	callbacksActive |= (1 << numFilters);
}

//+=====================================================================================================================
void  CANListener::detachGeneralHandler ()
{
	callbacksActive &= ~(1 << numFilters);
}

//+=====================================================================================================================
bool  CANListener::isCallbackActive (int callback)
{
    return (callbacksActive & (1 << callback)) ? true : false ;
}

//+=====================================================================================================================
// An empty version so that the linker doesn't complain that no implementation exists.
//
void  CANListener::gotFrame (CAN_FRAME* frame,  int mailbox)
{
	(void)frame;
	(void)mailbox;
}

//**********************************************************************************************************************
//**********************************************************************************************************************

//+=====================================================================================================================
// Some functions in CAN_COMMON are declared = 0 which means you HAVE to reimplement them or the compiler will puke.
// Some functions are optional and they've got blank or very simple implementations below.
//
CAN_COMMON::CAN_COMMON (int numFilt)
{
    numFilters = numFilt;

	// !!! what's going on with this missing malloc() ??
	// !!! oh, I see, we declare a fixed size array of 16 callbacks
	// !!! typdef the function pointer and do this properly/readably
    // Official entry for the worst, most convoluted looking C++ line ever written.
    // Dynamically allocate enough space for the function pointers with a hideous malloc call.
    //cbCANFrame = malloc(4 * numFilters);
    memset(cbCANFrame, 0, sizeof(void*) * numFilters);

    cbGeneral = 0;
    enablePin = 255;
    for (int  i = 0;  i < SIZE_LISTENERS;  listener[i++] = 0) ;
}

//+=====================================================================================================================
uint32_t  CAN_COMMON::begin ()
{
	return init(CAN_DEFAULT_BAUD);
}

//+=============================================================================
uint32_t  CAN_COMMON::begin (uint32_t baud)
{
	return init(baud);
}

//+=============================================================================
uint32_t  CAN_COMMON::begin (uint32_t baud,  uint8_t enPin)
{
	enablePin = enPin;
    return init(baud);
}

//+=====================================================================================================================
uint32_t  CAN_COMMON::getBusSpeed ()
{
	return busSpeed;
}

//+=====================================================================================================================
bool  CAN_COMMON::attachObj (CANListener* listener)
{
	// Use the first available listener
	for (int  i = 0;  i < SIZE_LISTENERS;  i++)
		if (this->listener[i] == NULL) {
			this->listener[i] = listener;
			listener->initialize();  // !!! get initialize to return its own pointer so we can move this up a line
			return true;
		}
	return false;
}

//+=====================================================================================================================
bool  CAN_COMMON::detachObj (CANListener* listener)
{
	for (int  i = 0;  i < SIZE_LISTENERS;  i++)
		if (this->listener[i] == listener) {
			this->listener[i] = NULL;
			return true;
		}
	return false;
}

//+=====================================================================================================================
// Set default callback for mailboxes with no registered callback
// If this function is used to set up a callback then no buffering of frames will ever take place.
//
void  CAN_COMMON::setGeneralCallback (void (*cb)(CAN_FRAME*))
{
	cbGeneral = cb;
}

//+=====================================================================================================================
// Set up a callback for a specific mailbox
//
// !!! needs to return bool
void  CAN_COMMON::setCallback (uint8_t mailbox,  void (*cb)(CAN_FRAME*))
{
	if (mailbox >= numFilters)  return ;//false;
	cbCANFrame[mailbox] = cb;
	return ;//true;
}

//+=============================================================================
// !!! wrapper for setCallback !?
void  CAN_COMMON::attachCANInterrupt (uint8_t mailBox,  void (*cb)(CAN_FRAME*))
{
	setCallback(mailBox, cb);
}

//+=====================================================================================================================
// !!! needs to return bool
void  CAN_COMMON::detachCANInterrupt (uint8_t mailBox)
{
	if (mailbox >= numFilters)  return ;//false;
	cbCANFrame[mailBox] = 0;
	return ;//true;
}

//+=====================================================================================================================
int  CAN_COMMON::setRXFilter (uint32_t id,  uint32_t mask,  bool extended)
{
    return _setFilter(id, mask, extended);
}

//+============================================================================
int  CAN_COMMON::setRXFilter (uint8_t mailbox,  uint32_t id,  uint32_t mask,  bool extended)
{
    return _setFilterSpecific(mailbox, id, mask, extended);
}

//+=====================================================================================================================
int  CAN_COMMON::watchFor ()
{
	if (id <= 0x7FF)  return setRXFilter(0, 0, false) ;
	else              return setRXFilter(0, 0, true ) ;
}

//+=============================================================================
// Let a single frame ID through.
// Automatic determination of extended & mask
//
int  CAN_COMMON::watchFor (uint32_t id)
{
	if (id <= 0x7FF)  return setRXFilter(id,      0x7FF, false) ;
	else              return setRXFilter(id, 0x1FFFFFFF, true ) ;
}

//+=============================================================================
// Allow a range of IDs through based on mask.
// Automatic determination of extended (only)
//
int  CAN_COMMON::watchFor (uint32_t id,  uint32_t mask)
{
	if (id <= 0x7FF)  return setRXFilter(id, mask, false) ;
	else              return setRXFilter(id, mask, true ) ;
}

//+=============================================================================
// Makes sure that the range from id1 to id2 is let through.
// This might open the floodgates if you aren't careful.
// There are undoubtedly better ways to do this, but this way seems to work.
// It'll be kind of slow if you try to let a huge span through though.
//
// Here is a quick overview of the theory behind these calculations:
//   We start with mask set to 11 or 29 set bits (all 1's)
//   and id set to the lowest ID in the range.
//
//   We then loop through every single ID possible in the range.
//   For each ID we AND with the current ID.
//   At the end only bits that never changed and were 1's will still be 1's.
//   This yields the ID we can match against to let these frames through.
//
//   The mask is calculated by finding the bitfield difference between the lowest ID and the current ID.
//   This calculation will be 1 anywhere the bits were different.
//   We invert this so that it is 1 anywhere the bits where the same.
//   Then we AND with the current Mask.
//
//   At the end the mask will be 1 anywhere the bits never changed.
//   ...This is the perfect mask.
//
//
int  CAN_COMMON::watchForRange (uint32_t idLo,  uint32_t idHi)
{
	uint32_t  id;
	uint32_t  mask;

	// Ensure id1 < id2
	if (idLo > idHi) {
		uint32_t  tmp = idLo;
		idLo = idHi;
		idHi = tmp;
	}

	id   = id1;
	mask = (id2 <= 0x7FF) ? 0x7FF : 0x1FFFFFFF ;
	for (uint32_t  c = id1;  c <= id2;  c++) {
		id   &= c;
		mask &= (~(id1 ^ c)) & 0x1FFFFFFF;
	}

	// output of the above crazy loop is actually the end result.
	return watchFor(id, mask);
}
