#ifndef _CAN_COMMON_
#define _CAN_COMMON_

#include <Arduino.h>

//----------------------------------------------------------------------------------------------------------------------
// #defines for standard baud rates
//
#ifdef CAN_BPS_500K  // !!! why are we doing this?? - what else defines them?
#	undef CAN_BPS_1000K
#	undef CAN_BPS_800K
#	undef CAN_BPS_500K
#	undef CAN_BPS_250K
#	undef CAN_BPS_125K
#	undef CAN_BPS_50K
#	undef CAN_BPS_33333
#	undef CAN_BPS_25K
#	undef CAN_BPS_10K
#	undef CAN_BPS_5K
#endif

#define CAN_BPS_1000K	(1000000ul)
#define CAN_BPS_800K	(800000ul)
#define CAN_BPS_500K	(500000ul)
#define CAN_BPS_250K	(250000ul)
#define CAN_BPS_125K	(125000ul)
#define CAN_BPS_50K		(50000ul)
#define CAN_BPS_33333	(33333ul)
#define CAN_BPS_25K		(25000ul)
#define CAN_BPS_10K		(10000ul)
#define CAN_BPS_5K		(5000ul)

#define CAN_DEFAULT_BAUD  CAN_BPS_250K  // !!! CAN_BPS_DFLT

#define SIZE_LISTENERS	4 //number of classes that can register as listeners with this class

//----------------------------------------------------------------------------------------------------------------------
// This structure presupposes little endian mode.
// If you use it on a big endian processor you're going to have a bad time.
//
// !!! this simple doesn't work 0x0102030405060708 -> {8,7,6,5,4,3,2,1}
// !!! This whole idea needs to be revisited - probably with MACROs
typedef
	union {
		uint64_t value;
		struct {
			uint32_t low;
			uint32_t high;
		};
		struct {
			uint16_t s0;
			uint16_t s1;
			uint16_t s2;
			uint16_t s3;
		};
		uint8_t byte[8];
		uint8_t bytes[8]; // !!! lose it
	}
BytesUnion;  // !!! bytesUnion_t

//----------------------------------------------------------------------------------------------------------------------
/*
// !!! I propose this replacement

typedef
  enum canDir {
    CDIR_RX = 0,
    CDIR_TX = 1,
  }
canDir_t;

typedef
  struct canFrame {
    // Defined in a strange order to help the compiler optimise data storage
    uint32_t  mS;       // mS - the CAN timer is too small, save the due timestamp as well!
    uint32_t  aid;      // Arbitration ID
    uint32_t  fid;      // Family ID
    uint8_t   data[8];  // Payload
    uint16_t  ts;       // Timestamp
    uint8_t   dlc;      // Length {0..8}
    uint8_t   pri;      // Priority {0..15} - Only used when transmitting frames
    bool      ext;      // Extended
    bool      rtr;      // Remote Tranmit Request
    uint8_t   bus;      // CAN Bus number   - Used by data logging systems
    canDir_t  dir;      // CDIR_RX, CDIR_TX - Used by data logging systems
  }
canFrame_t;

*/
typedef
	struct {
		uint32_t    id;        // EID if ide set, SID otherwise
		uint32_t    fid;       // family ID
		uint8_t     rtr;       // Remote Transmission Request
		uint8_t     priority;  // Priority but only important for TX frames and then only for special uses.
		uint8_t     extended;  // Extended ID flag
		uint16_t    time;      // CAN timer value when mailbox message was received.
		uint8_t     length;    // Number of data bytes
		BytesUnion  data;      // 64 bits - lots of ways to access it.
	}
CAN_FRAME; // !!! canFrame_t

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class CANListener
{
public:
	CANListener                () ;  // Constructor

	void  initialize           () ;
	void  setNumFilters        (int numFilt) ;

	void  attachMBHandler      (uint8_t mailBox) ;
	void  detachMBHandler      (uint8_t mailBox) ;

	void  attachGeneralHandler () ;
	void  detachGeneralHandler () ;

	bool  isCallbackActive     (int callback) ;

	virtual void  gotFrame     (CAN_FRAME* frame, int mailbox) ;

private:
	int  callbacksActive; // Bitfield of active callbacks - Only for object-oriented callbacks
	int  numFilters;      // filters, mailboxes, whichever, how many do we have? !!! yeah, pick a naming convention
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Abstract function that mostly just sets an interface that all descendants must implement
//
class CAN_COMMON
{
public:
	CAN_COMMON (int numFilt) ;  // Constructor

	//Public API that needs to be re-implemented by subclasses
	virtual  uint32_t  init               (uint32_t ul_baudrate) = 0 ;

	virtual  int       _setFilter         (/*-----------*/   uint32_t id,  uint32_t mask,  bool extended) = 0 ;
	virtual  int       _setFilterSpecific (uint8_t mailbox,  uint32_t id,  uint32_t mask,  bool extended) = 0 ;

	virtual  uint32_t  beginAutoSpeed     ()                     = 0 ;
	virtual  uint32_t  set_baudrate       (uint32_t ul_baudrate) = 0 ;

	virtual  void      setListenOnlyMode  (bool state)           = 0 ;

	virtual  void      enable             ()                     = 0 ;
	virtual  void      disable            ()                     = 0 ;

	virtual  bool      sendFrame          (CAN_FRAME& txFrame)   = 0 ;

	virtual  bool      rx_avail           ()                     = 0 ;
	virtual  uint16_t  available          ()                     = 0 ; // Like rx_avail, but returns count of waiting frames
	virtual  uint32_t  get_rx_buff        (CAN_FRAME& msg)       = 0 ;

	// Public API common to all subclasses - don't need to be re-implemented
	// wrapper for syntactic sugar reasons
	// NB. Functions CANNOT be BOTH virtual AND overloaded
	inline  uint32_t  read (CAN_FRAME &msg)  { return get_rx_buff(msg); }
	int       watchFor      () ;                                        // Promiscuous mode
	int       watchFor      (uint32_t id) ;                             // Specific AID (auto-std|ext)
	int       watchFor      (uint32_t id,  uint32_t mask) ;             // AID_Mask (auto-std|ext)
	int       watchFor      (uint32_t id,  uint32_t mask,  bool ext) ;  // AID_Mask (specify std|ext)
	int       watchForRange (uint32_t idLo,  uint32_t idHi);            // Auto-AID_Mask and auto-std|ext

	uint32_t  begin () ;
	uint32_t  begin (uint32_t baudrate) ;
	uint32_t  begin (uint32_t baudrate,  uint8_t enPin) ;

	uint32_t  getBusSpeed () ;

	int       setRXFilter (/*-----------*/   uint32_t id,  uint32_t mask,  bool extended);
	int       setRXFilter (uint8_t mailbox,  uint32_t id,  uint32_t mask,  bool extended);

	bool      attachObj (CANListener* listener);
	bool      detachObj (CANListener* listener);

	void      setGeneralCallback (void (*cb)(CAN_FRAME*));  // !!! typdef these function pointers!
	void      attachCANInterrupt (void (*cb)(CAN_FRAME*))  { setGeneralCallback(cb); }  // !!! WHY, Just WHY?

	void      setCallback        (uint8_t mailbox,  void (*cb)(CAN_FRAME*));
	void      attachCANInterrupt (uint8_t mailBox,  void (*cb)(CAN_FRAME*));
	void      detachCANInterrupt (uint8_t mailBox);

protected:
	void (*cbGeneral)     (CAN_FRAME*); // General callback if no per-mailbox or per-filter entries matched
	void (*cbCANFrame[16])(CAN_FRAME*); // Array of function pointers - disgusting syntax though. !!! then typedef it!

	CANListener*  listener[SIZE_LISTENERS];
	uint32_t      busSpeed;
	int           numFilters;
	int           enablePin;
};

#endif
