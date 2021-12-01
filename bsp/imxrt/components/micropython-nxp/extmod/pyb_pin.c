#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/virtpin.h"
#include "pin_defs_mcu.h"

/// \moduleref pyb
/// \class Pin - control I/O pins
///
/// A pin is the basic object to control I/O pins.  It has methods to set
/// the mode of the pin (input, output, etc) and methods to get and set the
/// digital logic level.  For analog control of a pin, see the ADC class.
///
/// Usage Model:
///
/// All Board Pins are predefined as pyb.Pin.board.Name
///
///     x1_pin = pyb.Pin.board.X1
///
///     g = pyb.Pin(pyb.Pin.board.X1, pyb.Pin.IN)
///
/// CPU pins which correspond to the board pins are available
/// as `pyb.cpu.Name`. For the CPU pins, the names are the port letter
/// followed by the pin number. On the PYBv1.0, `pyb.Pin.board.X1` and
/// `pyb.Pin.cpu.B6` are the same pin.
///
/// You can also use strings:
///
///     g = pyb.Pin('X1', pyb.Pin.OUT_PP)
///
/// Users can add their own names:
///
///     MyMapperDict = { 'LeftMotorDir' : pyb.Pin.cpu.C12 }
///     pyb.Pin.dict(MyMapperDict)
///     g = pyb.Pin("LeftMotorDir", pyb.Pin.OUT_OD)
///
/// and can query mappings
///
///     pin = pyb.Pin("LeftMotorDir")
///
/// Users can also add their own mapping function:
///
///     def MyMapper(pin_name):
///        if pin_name == "LeftMotorDir":
///            return pyb.Pin.cpu.A0
///
///     pyb.Pin.mapper(MyMapper)
///
/// So, if you were to call: `pyb.Pin("LeftMotorDir", pyb.Pin.OUT_PP)`
/// then `"LeftMotorDir"` is passed directly to the mapper function.
///
/// To summarise, the following order determines how things get mapped into
/// an ordinal pin number:
///
/// 1. Directly specify a pin object
/// 2. User supplied mapping function
/// 3. User supplied mapping (object must be usable as a dictionary key)
/// 4. Supply a string which matches a board pin
/// 5. Supply a string which matches a CPU port/pin
///
/// You can set `pyb.Pin.debug(True)` to get some debug information about
/// how a particular object gets mapped to a pin.

// Pin class variables

#define REG_READ32(reg)  (*((volatile uint32_t *)(reg)))
	

STATIC bool pin_class_debug = false;
const mp_obj_type_t pyb_pin_type;
const mp_obj_type_t pin_board_pins_obj_type;
const mp_obj_type_t pin_cpu_pins_obj_type;
const pin_obj_t *pin_find_named_pin(const mp_obj_dict_t *named_pins, mp_obj_t name);

void pyb_pin_init(void) {
    MP_STATE_PORT(pin_class_mapper) = mp_const_none;
    MP_STATE_PORT(pin_class_map_dict) = mp_const_none;
    pin_class_debug = false;
}

uint32_t pin_get_mode(const pin_obj_t *pin) {
	McuPinCfgReg_t t;
	t.v32 = REG_READ32(pin->cfgReg);
	return t.v32;
}

// Returns the pin pullup/pulldown. The value returned by this macro should
// be one of GPIO_NOPULL, GPIO_PULLUP, or GPIO_PULLDOWN.

uint32_t pin_get_pull(const pin_obj_t *pin) {
	uint32_t t = REG_READ32(pin->cfgReg) & (0x1F << 12);
	return t;
}

// Returns the af (alternate function) index currently set for a pin.

uint32_t pin_get_af(const pin_obj_t *pin) {
	uint32_t t = REG_READ32(pin->afReg);
	uint32_t afNdx = t & 7;
	uint32_t i;
	for (i=0; i<pin->num_af; i++) {
		if (pin->af[i].idx == afNdx) {
			if (0 != pin->af[i].inSelReg) {
				if (REG_READ32(pin->af[i].inSelReg) != pin->af[i].inSelVal) {
					afNdx |= 0x10;	// this means the AF is not selected by 2nd level of muxing (INSEL/DAISY)
				}
			}
			break;
		}
	}
	return afNdx;
}

void mp_hal_gpio_clock_enable(uint32_t portNum) {
	 // i.MX RT's GPIO port starts from 1, and clock gate is not ordered in GPIO order
	#ifdef SOC_MIMXRT1176DVMMA
	// all the gpio group has only one gate
	CLOCK_EnableClock(kCLOCK_Gpio);
	#else
	const static clock_ip_name_t tab[] = {
	(clock_ip_name_t)0, kCLOCK_Gpio1, kCLOCK_Gpio2, kCLOCK_Gpio3, 
#if defined(SOC_IMXRT1060_SERIES)	||	defined(SOC_MIMXRT1050_SERIES)
	kCLOCK_Gpio4,
#endif		
	kCLOCK_Gpio5
	};
	CLOCK_EnableClock(tab[portNum]); 
	#endif

}

static inline void mp_hal_pin_low(const pin_obj_t *pPin) {
	GPIO_PinWrite(pPin->gpio, pPin->pin, 0);
}

static inline void mp_hal_pyb_pin_high(const pin_obj_t *pPin) {
	GPIO_PinWrite(pPin->gpio, pPin->pin, 1);
}
static inline void mp_hal_pin_toggle(const pin_obj_t *pPin)
{
	uint32_t a, pinNdx = pPin->pin;
	a = (0 == GPIO_PinRead(pPin->gpio, pinNdx) );
	GPIO_PinWrite(pPin->gpio, pinNdx, a);
	
}

static inline GPIO_Type * _find_gpio(const pin_obj_t *p){
	GPIO_Type *gps[] = {0, GPIO1, GPIO2, GPIO3, 0, GPIO5,};
	if (p->port > 5)
		while(1);
	return gps[p->port];
}

void mp_hal_pin_open_set(const pin_obj_t *p, int mode)
{
	if (mode == PIN_MODE_INPUT)
	{
		mp_hal_ConfigGPIO(p,GPIO_MODE_INPUT,0);
	}
	else if (mode == PIN_MODE_OUTPUT)
	{
		mp_hal_ConfigGPIO(p,GPIO_MODE_OUTPUT_PP,0);
	}
	else if (mode == PIN_MODE_OUTPUT_OD)
	{
		mp_hal_ConfigGPIO(p,GPIO_MODE_OUTPUT_OD,0);
	}
}

void mp_hal_pin_write(const pin_obj_t *pPin, int value) {
	GPIO_PinWrite(pPin->gpio, pPin->pin, value);
}

uint8_t mp_hal_pin_read(const pin_obj_t *p) {
	return GPIO_PinReadPadStatus(_find_gpio(p), (p)->pin);
}

char* mp_hal_pin_get_name(const pin_obj_t *p) {
	size_t qstrLen;
	return (char*)qstr_data(p->name, &qstrLen);
}

const pin_af_obj_t *pin_find_af(const pin_obj_t *pin, uint8_t fn) {
    const pin_af_obj_t *af = pin->af;
    for (uint32_t i = 0; i < pin->num_af; i++, af++) {
        if (af->fn == fn) {
            return af;
        }
    }
    return NULL;
}


void mp_hal_pin_config(const pin_obj_t *p, const pin_af_obj_t *af, uint32_t alt, uint32_t padCfgVal ) {
	uint32_t isInputPathForcedOn = 0;
	padCfgVal &= ~(1UL<<31);	// clear MSb, as it is used to mark input/output for GPIO
	CLOCK_EnableClock(kCLOCK_Iomuxc);
	
	if (alt == PIN_ALT_NC) 
		alt = REG_READ32(p->afReg) & 7;
	isInputPathForcedOn =  (alt == 5 || (padCfgVal | 1<<11) );	// Alt 5 is GPIO or OD mode is selected
	IOMUXC_SetPinMux(p->afReg, alt, af->inSelReg, af->inSelVal, p->cfgReg, isInputPathForcedOn);
	IOMUXC_SetPinConfig(p->afReg,alt,af->inSelReg, af->inSelVal, p->cfgReg, padCfgVal & (~(1<<4)));
}

bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin, uint32_t padCfg,  uint8_t fn) {
    const pin_af_obj_t *af = pin_find_af(pin, fn);
    if (af == NULL) {
        return false;
    }
	mp_hal_pin_config(pin, af, af->idx, padCfg);
    return true;
}

void mp_hal_ConfigGPIO(const pin_obj_t *p, uint32_t gpioModeAndPadCfg, uint32_t isInitialHighForOutput)
{
	GPIO_Type *pGPIO = p->gpio;
	uint32_t pinMask = 1 << p->pin;
	mp_hal_gpio_clock_enable(p->port);
	pGPIO->IMR &= ~pinMask;	 // disable pin IRQ
	if (gpioModeAndPadCfg & (GPIO_PAD_OUTPUT_MASK)) {
		// output
		if (isInitialHighForOutput)
			pGPIO->DR |= pinMask;
		else
			pGPIO->DR &= ~pinMask;
		pGPIO->GDIR |= pinMask;
		
	} else {
		// input
		pGPIO->GDIR &= ~pinMask;
	}
	mp_hal_pin_config_alt((mp_hal_pin_obj_t)p, gpioModeAndPadCfg, AF_FN_GPIO);
}
// C API used to convert a user-supplied pin name into an ordinal pin number.
const pin_obj_t *pin_find(mp_obj_t user_obj) {
    const pin_obj_t *pin_obj;

    // If a pin was provided, then use it
    if (MP_OBJ_IS_TYPE(user_obj, &pyb_pin_type)) {
        pin_obj = user_obj;
        if (pin_class_debug) {
            printf("Pin map passed pin ");
            mp_obj_print((mp_obj_t)pin_obj, PRINT_STR);
            printf("\n");
        }
        return pin_obj;
    }

    if (MP_STATE_PORT(pin_class_mapper) != mp_const_none) {
        pin_obj = mp_call_function_1(MP_STATE_PORT(pin_class_mapper), user_obj);
        if (pin_obj != mp_const_none) {
            if (!MP_OBJ_IS_TYPE(pin_obj, &pyb_pin_type)) {
                nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Pin.mapper didn't return a Pin object"));
            }
            if (pin_class_debug) {
                printf("Pin.mapper maps ");
                mp_obj_print(user_obj, PRINT_REPR);
                printf(" to ");
                mp_obj_print((mp_obj_t)pin_obj, PRINT_STR);
                printf("\n");
            }
            return pin_obj;
        }
        // The pin mapping function returned mp_const_none, fall through to
        // other lookup methods.
    }

    if (MP_STATE_PORT(pin_class_map_dict) != mp_const_none) {
        mp_map_t *pin_map_map = mp_obj_dict_get_map(MP_STATE_PORT(pin_class_map_dict));
        mp_map_elem_t *elem = mp_map_lookup(pin_map_map, user_obj, MP_MAP_LOOKUP);
        if (elem != NULL && elem->value != NULL) {
            pin_obj = elem->value;
            if (pin_class_debug) {
                printf("Pin.map_dict maps ");
                mp_obj_print(user_obj, PRINT_REPR);
                printf(" to ");
                mp_obj_print((mp_obj_t)pin_obj, PRINT_STR);
                printf("\n");
            }
            return pin_obj;
        }
    }

    // See if the pin name matches a board pin
    pin_obj = pin_find_named_pin(&pin_board_pins_locals_dict, user_obj);
    if (pin_obj) {
        if (pin_class_debug) {
            printf("Pin.board maps ");
            mp_obj_print(user_obj, PRINT_REPR);
            printf(" to ");
            mp_obj_print((mp_obj_t)pin_obj, PRINT_STR);
            printf("\n");
        }
        return pin_obj;
    }

    // See if the pin name matches a cpu pin
    pin_obj = pin_find_named_pin(&pin_cpu_pins_locals_dict, user_obj);
    if (pin_obj) {
        if (pin_class_debug) {
            printf("Pin.cpu maps ");
            mp_obj_print(user_obj, PRINT_REPR);
            printf(" to ");
            mp_obj_print((mp_obj_t)pin_obj, PRINT_STR);
            printf("\n");
        }
        return pin_obj;
    }
#ifdef BSP_USING_74HC595
	pin_obj = pin_find_named_pin(&pin_extend_pins_locals_dict, user_obj);
    if (pin_obj) {
        if (pin_class_debug) {
            printf("Pin.cpu maps ");
            mp_obj_print(user_obj, PRINT_REPR);
            printf(" to ");
            mp_obj_print((mp_obj_t)pin_obj, PRINT_STR);
            printf("\n");
        }
        return pin_obj;
    }
#endif	

    nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "pin '%s' not a valid pin identifier", mp_obj_str_get_str(user_obj)));
}

/// \method __str__()
/// Return a string describing the pin object.
#define _INC_PRINT(...) do { \
	snprintf(s + sNdx, sLenRem, __VA_ARGS__); \
	sNdx = strlen(s); \
	sLenRem = sizeof(s) - sNdx; }while(0)

STATIC void pin_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pin_obj_t *self = self_in;
	char s[144];
	const char* ppSpd[] = {"50M", "100M", "100M", "200M"};
	const char* ppPull[] = {"pDn100K", "pUp47K", "pUp100K", "pUp22K"};
	uint32_t sNdx = 0, sLenRem = sizeof(s);

	McuPinCfgReg_t cfg;
	McuPinMuxReg_t mux;
	uint32_t afNdx;
	const char *pName, *pBoardName;
	char levels[2] = "LH";
	size_t qstrLen;
	pName = (const char*) qstr_data(self->name, &qstrLen);	
	pBoardName = (const char*) qstr_data(self->board_name, &qstrLen);	
    cfg.v32 = REG_READ32(self->cfgReg);
	mux.v32 = REG_READ32(self->afReg);
	uint32_t inLevel = levels[(((GPIO_Type *)(self->gpio))->PSR >> self->pin) & 1];
	uint32_t drvLevel = levels[(((GPIO_Type *)(self->gpio))->DR >> self->pin) & 1];
	if (mux.b04_1_inForceOn)
		_INC_PRINT("Pin %s (GPIO%d.%02d %s), PAD:%c, MUX_CFG=0x%02x, PAD_CFG=0x%05x:\n", pBoardName, self->port, self->pin, pName, inLevel, mux.v32, cfg.v32);
	else
		_INC_PRINT("Pin %s (GPIO%d.%02d %s), PAD:-, MUX_CFG=0x%02x, PAD_CFG=0x%05x:\n", pBoardName, self->port, self->pin, pName, mux.v32, cfg.v32);	
	afNdx = pin_get_af(self);
	if (cfg.b11_1_OD_isOD)
		_INC_PRINT("OD, ");
	else
		_INC_PRINT("--, ");
    if (!(cfg.b12_1_PKE_digiInEn)) {
        // analog
		_INC_PRINT("Analog/Hiz\n");
    } else {
    	_INC_PRINT("Digital, mux=%d, ", afNdx & 7);
		if (mux.b04_1_inForceOn)
			_INC_PRINT("In %c, ", inLevel);
		else
			_INC_PRINT("----, ");
    	if (afNdx == 5) {
			_INC_PRINT("GPIO:");
			if (((GPIO_Type *)(self->gpio))->GDIR & (1<<self->pin))
				_INC_PRINT("OUT %c, ", drvLevel);
			else
				_INC_PRINT(" IN, ");
    	}
    }
	if (cfg.b00_1_SRE_isFastSlew)
		_INC_PRINT("Fast slew, ");
	else
		_INC_PRINT("Slow slew, ");

	_INC_PRINT("drive=%d/8, ", cfg.b03_3_DSE_driveStrength);
	_INC_PRINT("%s SPD, ", ppSpd[cfg.b06_2_Speed]);
	if (cfg.b13_1_PUE_keepOrPull == 0)
		_INC_PRINT("keeper, ");
	else
		_INC_PRINT("%s, ", ppPull[cfg.b14_2_PUS_PullSel]);

	if (cfg.b16_1_HYS)
		_INC_PRINT("HYS, ");
	else
		_INC_PRINT("---, ");

	if (afNdx & 0x10) {
		_INC_PRINT("not selected! ");
	}

	mp_printf(print, "%s\n", s);
}

STATIC mp_obj_t pin_obj_init_helper(const pin_obj_t *pin, mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args);

/// \classmethod \constructor(id, ...)
/// Create a new Pin object associated with the id.  If additional arguments are given,
/// they are used to initialise the pin.  See `init`.
mp_obj_t pyb_pin_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // Run an argument through the mapper and return the result.
    const pin_obj_t *pin = pin_find(args[0]);

    if (n_args > 1 || n_kw > 0) {
        // pin mode given, so configure this GPIO
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        pin_obj_init_helper(pin, n_args - 1, args + 1, &kw_args);
    }

    return (mp_obj_t)pin;
}

// fast method for getting/setting pin value
STATIC mp_obj_t pin_call(mp_obj_t self_in, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    pin_obj_t *self = self_in;
    if (n_args == 0) {
        // get pin
        return MP_OBJ_NEW_SMALL_INT(mp_hal_pin_read(self));
    } else {
        // set pin
        mp_hal_pin_write(self, mp_obj_is_true(args[0]));
        return mp_const_none;
    }
}

/// \classmethod mapper([fun])
/// Get or set the pin mapper function.
STATIC mp_obj_t pin_mapper(mp_uint_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        MP_STATE_PORT(pin_class_mapper) = args[1];
        return mp_const_none;
    }
    return MP_STATE_PORT(pin_class_mapper);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_mapper_fun_obj, 1, 2, pin_mapper);
STATIC MP_DEFINE_CONST_CLASSMETHOD_OBJ(pin_mapper_obj, (mp_obj_t)&pin_mapper_fun_obj);

/// \classmethod dict([dict])
/// Get or set the pin mapper dictionary.
STATIC mp_obj_t pin_map_dict(mp_uint_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        MP_STATE_PORT(pin_class_map_dict) = args[1];
        return mp_const_none;
    }
    return MP_STATE_PORT(pin_class_map_dict);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_map_dict_fun_obj, 1, 2, pin_map_dict);
STATIC MP_DEFINE_CONST_CLASSMETHOD_OBJ(pin_map_dict_obj, (mp_obj_t)&pin_map_dict_fun_obj);

/// \classmethod af_list()
/// Returns an array of alternate functions available for this pin.
STATIC mp_obj_t pin_af_list(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    mp_obj_t result = mp_obj_new_list(0, NULL);

    const pin_af_obj_t *af = self->af;
    for (mp_uint_t i = 0; i < self->num_af; i++, af++) {
        mp_obj_list_append(result, (mp_obj_t)af);
    }
    return result;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_af_list_obj, pin_af_list);

/// \classmethod debug([state])
/// Get or set the debugging state (`True` or `False` for on or off).
STATIC mp_obj_t pin_debug(mp_uint_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        pin_class_debug = mp_obj_is_true(args[1]);
        return mp_const_none;
    }
    return mp_obj_new_bool(pin_class_debug);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_debug_fun_obj, 1, 2, pin_debug);
STATIC MP_DEFINE_CONST_CLASSMETHOD_OBJ(pin_debug_obj, (mp_obj_t)&pin_debug_fun_obj);

// init(mode, pull=None, af=-1, *, value, alt, inv=0)
typedef struct _pin_init_t{
		mp_arg_val_t mode, value, alt, fastslew, hys, pad_expert_cfg;
}pin_init_t;
STATIC mp_obj_t pin_obj_init_helper(const pin_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_INT },
		// embedded in "mode" { MP_QSTR_pull, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        { MP_QSTR_value, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true}},
        { MP_QSTR_alt, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5}},
		{ MP_QSTR_fastslew, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
		{ MP_QSTR_hys, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
		// if this arg is specified, it overrides mode & hys. User must have i.mx RT105 pad cfg h/w know how to use (SW_PAD_CTL_<PAD>)
		{ MP_QSTR_pad_expert_cfg, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},	
    };
	pin_init_t args;
    // parse args
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, (mp_arg_val_t*)&args);
	mp_uint_t alt = args.alt.u_int;
	mp_uint_t hys = (args.hys.u_bool != 0) << 16;	// HYS bit
	mp_uint_t slew = (args.fastslew.u_bool != 0) << 0;	// SRE bit
	mp_uint_t padCfg;
    if (args.pad_expert_cfg.u_obj != MP_OBJ_NULL) {
		padCfg = (int)args.pad_expert_cfg.u_obj;
    } else {
		padCfg = args.mode.u_int | hys | slew;		
    }
	if (alt == 5) {
		// config GPIO
		mp_hal_ConfigGPIO(self, padCfg, args.value.u_int);
	} else {
		mp_hal_pin_config_alt((mp_hal_pin_obj_t)self, padCfg, alt);
	}
    return mp_const_none;
}

STATIC mp_obj_t pin_obj_init(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return pin_obj_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
MP_DEFINE_CONST_FUN_OBJ_KW(pin_init_obj, 1, pin_obj_init);

/// \method value([value])
/// Get or set the digital logic level of the pin:
///
///   - With no argument, return 0 or 1 depending on the logic level of the pin.
///   - With `value` given, set the logic level of the pin.  `value` can be
///   anything that converts to a boolean.  If it converts to `True`, the pin
///   is set high, otherwise it is set low.
STATIC mp_obj_t pin_value(mp_uint_t n_args, const mp_obj_t *args) {
    return pin_call(args[0], n_args - 1, 0, args + 1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pin_value_obj, 1, 2, pin_value);

STATIC mp_obj_t pin_off(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    mp_hal_pin_low(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_off_obj, pin_off);

STATIC mp_obj_t pin_on(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    mp_hal_pyb_pin_high(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_on_obj, pin_on);

/// \method name()
/// Get the pin name.
STATIC mp_obj_t pin_name(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    return MP_OBJ_NEW_QSTR(self->name);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_name_obj, pin_name);

/// \method names()
/// Returns the cpu and board names for this pin.
STATIC mp_obj_t pin_names(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    mp_obj_t result = mp_obj_new_list(0, NULL);
    mp_obj_list_append(result, MP_OBJ_NEW_QSTR(self->name));

    mp_map_t *map = mp_obj_dict_get_map((mp_obj_t)&pin_board_pins_locals_dict);
    mp_map_elem_t *elem = map->table;

    for (mp_uint_t i = 0; i < map->used; i++, elem++) {
        if (elem->value == self) {
            mp_obj_list_append(result, elem->key);
        }
    }
    return result;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_names_obj, pin_names);

/// \method port()
/// Get the pin port.
STATIC mp_obj_t pin_port(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    return MP_OBJ_NEW_SMALL_INT(self->port);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_port_obj, pin_port);

/// \method pin()
/// Get the pin number.
STATIC mp_obj_t pin_pin(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    return MP_OBJ_NEW_SMALL_INT(self->pin);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_pin_obj, pin_pin);

/// \method gpio()
/// Returns the base address of the GPIO block associated with this pin.
STATIC mp_obj_t pin_gpio(mp_obj_t self_in) {
    pin_obj_t *self = self_in;
    return MP_OBJ_NEW_SMALL_INT((mp_int_t)self->gpio);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_gpio_obj, pin_gpio);

/// \method mode()
/// Returns the currently configured mode of the pin. The integer returned
/// will match one of the allowed constants for the mode argument to the init
/// function.
STATIC mp_obj_t pin_mode(mp_obj_t self_in) {
    return MP_OBJ_NEW_SMALL_INT(pin_get_mode(self_in));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_mode_obj, pin_mode);

/// \method pull()
/// Returns the currently configured pull of the pin. The integer returned
/// will match one of the allowed constants for the pull argument to the init
/// function.
STATIC mp_obj_t pin_pull(mp_obj_t self_in) {
    return MP_OBJ_NEW_SMALL_INT(pin_get_pull(self_in));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_pull_obj, pin_pull);

/// \method af()
/// Returns the currently configured alternate-function of the pin. The
/// integer returned will match one of the allowed constants for the af
/// argument to the init function.
STATIC mp_obj_t pin_af(mp_obj_t self_in) {
    return MP_OBJ_NEW_SMALL_INT(pin_get_af(self_in));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_af_obj, pin_af);

STATIC void pin_isr_handler(void *arg) {
    pin_obj_t *self = arg;
    mp_sched_schedule(self->pin_isr_cb, MP_OBJ_FROM_PTR(self));
}

// pin.irq(handler=None, trigger=IRQ_FALLING|IRQ_RISING)
STATIC mp_obj_t pin_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_handler, ARG_trigger, ARG_wake };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_trigger, MP_ARG_INT, {.u_int = PIN_IRQ_MODE_RISING} },
    };
    pin_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (n_args > 1 || kw_args->used != 0) {
        // configure irq
        self->pin_isr_cb = args[ARG_handler].u_obj;
        uint32_t trigger = args[ARG_trigger].u_int;

        rt_pin_mode(self->pin, PIN_MODE_INPUT_PULLUP);
        rt_pin_attach_irq(self->pin, trigger, pin_isr_handler, (void*)self);
        rt_pin_irq_enable(self->pin, PIN_IRQ_ENABLE);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pin_irq_obj, 1, pin_irq);

STATIC const mp_rom_map_elem_t pin_locals_dict_table[] = {
    // instance methods
    { MP_ROM_QSTR(MP_QSTR_init),    MP_ROM_PTR(&pin_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_value),   MP_ROM_PTR(&pin_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_off),     MP_ROM_PTR(&pin_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_on),      MP_ROM_PTR(&pin_on_obj) },
    // Legacy names as used by pyb.Pin
	{ MP_ROM_QSTR(MP_QSTR_irq),     MP_ROM_PTR(&pin_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_low),     MP_ROM_PTR(&pin_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_high),    MP_ROM_PTR(&pin_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_name),    MP_ROM_PTR(&pin_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_names),   MP_ROM_PTR(&pin_names_obj) },
    { MP_ROM_QSTR(MP_QSTR_af_list), MP_ROM_PTR(&pin_af_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_port),    MP_ROM_PTR(&pin_port_obj) },
    { MP_ROM_QSTR(MP_QSTR_pin),     MP_ROM_PTR(&pin_pin_obj) },
    { MP_ROM_QSTR(MP_QSTR_gpio),    MP_ROM_PTR(&pin_gpio_obj) },

	{ MP_ROM_QSTR(MP_QSTR_mode),    MP_ROM_PTR(&pin_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_pull),    MP_ROM_PTR(&pin_pull_obj) },
    { MP_ROM_QSTR(MP_QSTR_af),      MP_ROM_PTR(&pin_af_obj) },

    // class methods
    { MP_ROM_QSTR(MP_QSTR_mapper),  MP_ROM_PTR(&pin_mapper_obj) },
    { MP_ROM_QSTR(MP_QSTR_dict),    MP_ROM_PTR(&pin_map_dict_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),   MP_ROM_PTR(&pin_debug_obj) },

    // class attributes
    { MP_ROM_QSTR(MP_QSTR_board),   MP_ROM_PTR(&pin_board_pins_obj_type) },
    { MP_ROM_QSTR(MP_QSTR_cpu),     MP_ROM_PTR(&pin_cpu_pins_obj_type) },

    // class constants

	{ MP_ROM_QSTR(MP_QSTR_HIZ),        	MP_ROM_INT(IOPAD_IN_HIZ) },
    { MP_ROM_QSTR(MP_QSTR_ANALOG),      MP_ROM_INT(GPIO_MODE_ANALOG) },
    { MP_ROM_QSTR(MP_QSTR_IN),          MP_ROM_INT(GPIO_MODE_INPUT) },
	{ MP_ROM_QSTR(MP_QSTR_IN_PUP),		MP_ROM_INT(GPIO_MODE_INPUT_PUP) },
	{ MP_ROM_QSTR(MP_QSTR_IN_PUP_WEAK), MP_ROM_INT(GPIO_MODE_INPUT_PUP_WEAK) },
	{ MP_ROM_QSTR(MP_QSTR_IN_PDN),	    MP_ROM_INT(GPIO_MODE_INPUT_PDN) },

	{ MP_ROM_QSTR(MP_QSTR_OUT),     	MP_ROM_INT(GPIO_MODE_OUTPUT_PP) },
	{ MP_ROM_QSTR(MP_QSTR_OUT_WEAK),    MP_ROM_INT(GPIO_MODE_OUTPUT_PP_WEAK) },
	{ MP_ROM_QSTR(MP_QSTR_OPEN_DRAIN),  MP_ROM_INT(GPIO_MODE_OUTPUT_OD) },
	{ MP_ROM_QSTR(MP_QSTR_OD_PUP),      MP_ROM_INT(GPIO_MODE_OUTPUT_OD_PUP) },
	// >>> below are custom cfg
	{ MP_ROM_QSTR(MP_QSTR_SLEW_FAST),   MP_ROM_INT(IOPAD_OUT_SLEW_FAST) },
	{ MP_ROM_QSTR(MP_QSTR_HYS),   		MP_ROM_INT(IOPAD_IN_HYST) },
	{ MP_ROM_QSTR(MP_QSTR_PULL_DOWN), MP_ROM_INT(GPIO_PULLDOWN) },
    { MP_ROM_QSTR(MP_QSTR_PULL_NONE), MP_ROM_INT(GPIO_NOPULL) },
    { MP_ROM_QSTR(MP_QSTR_PULL_UP),   MP_ROM_INT(GPIO_PULLUP) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_RISING), MP_ROM_INT(PIN_IRQ_MODE_RISING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_FALLING), MP_ROM_INT(PIN_IRQ_MODE_FALLING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_RISING_FALLING), MP_ROM_INT(PIN_IRQ_MODE_RISING_FALLING) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_LOW_LEVEL), MP_ROM_INT(PIN_IRQ_MODE_LOW_LEVEL) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_HIGH_LEVEL), MP_ROM_INT(PIN_IRQ_MODE_HIGH_LEVEL) },
};

STATIC MP_DEFINE_CONST_DICT(pin_locals_dict, pin_locals_dict_table);

STATIC mp_uint_t pin_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    (void)errcode;
    pin_obj_t *self = self_in;

    switch (request) {
        case MP_PIN_READ: {
            return mp_hal_pin_read(self);
        }
        case MP_PIN_WRITE: {
            mp_hal_pin_write(self, arg);
            return 0;
        }
    }
    return -1;
}

STATIC const mp_pin_p_t pin_pin_p = {
    .ioctl = pin_ioctl,
};

const mp_obj_type_t pyb_pin_type = {
    { &mp_type_type },
    .name = MP_QSTR_Pin,
    .print = pin_print,
    .make_new = pyb_pin_make_new,
    .call = pin_call,
    .protocol = &pin_pin_p,
    .locals_dict = (mp_obj_dict_t*)&pin_locals_dict,
};

/// \moduleref pyb
/// \class PinAF - Pin Alternate Functions
///
/// A Pin represents a physical pin on the microcprocessor. Each pin
/// can have a variety of functions (GPIO, I2C SDA, etc). Each PinAF
/// object represents a particular function for a pin.
///
/// Usage Model:
///
///     x3 = pyb.Pin.board.X3
///     x3_af = x3.af_list()
///
/// x3_af will now contain an array of PinAF objects which are availble on
/// pin X3.
///
/// For the pyboard, x3_af would contain:
///     [Pin.AF1_TIM2, Pin.AF2_TIM5, Pin.AF3_TIM9, Pin.AF7_USART2]
///
/// Normally, each peripheral would configure the af automatically, but sometimes
/// the same function is available on multiple pins, and having more control
/// is desired.
///
/// To configure X3 to expose TIM2_CH3, you could use:
///    pin = pyb.Pin(pyb.Pin.board.X3, mode=pyb.Pin.AF_PP, af=pyb.Pin.AF1_TIM2)
/// or:
///    pin = pyb.Pin(pyb.Pin.board.X3, mode=pyb.Pin.AF_PP, af=1)

/// \method __str__()
/// Return a string describing the alternate function.
STATIC void pin_af_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pin_af_obj_t *self = self_in;
    mp_printf(print, "Pin.%q", self->name);
}

/// \method index()
/// Return the alternate function index.
STATIC mp_obj_t pin_af_index(mp_obj_t self_in) {
    pin_af_obj_t *af = self_in;
    return MP_OBJ_NEW_SMALL_INT(af->idx);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_af_index_obj, pin_af_index);

/// \method name()
/// Return the name of the alternate function.
STATIC mp_obj_t pin_af_name(mp_obj_t self_in) {
    pin_af_obj_t *af = self_in;
    return MP_OBJ_NEW_QSTR(af->name);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_af_name_obj, pin_af_name);

/// \method reg()
/// Return the base register associated with the peripheral assigned to this
/// alternate function. For example, if the alternate function were TIM2_CH3
/// this would return stm.TIM2
STATIC mp_obj_t pin_af_reg(mp_obj_t self_in) {
    pin_af_obj_t *af = self_in;
    return MP_OBJ_NEW_SMALL_INT((mp_uint_t)af->reg);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pin_af_reg_obj, pin_af_reg);

STATIC const mp_rom_map_elem_t pin_af_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_index), MP_ROM_PTR(&pin_af_index_obj) },
    { MP_ROM_QSTR(MP_QSTR_name), MP_ROM_PTR(&pin_af_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg), MP_ROM_PTR(&pin_af_reg_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pin_af_locals_dict, pin_af_locals_dict_table);

const mp_obj_type_t pin_af_type = {
    { &mp_type_type },
    .name = MP_QSTR_PinAF,
    .print = pin_af_obj_print,
    .locals_dict = (mp_obj_dict_t*)&pin_af_locals_dict,
};



STATIC void pin_named_pins_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    pin_named_pins_obj_t *self = self_in;
    mp_printf(print, "<Pin.%q>", self->name);
}

const mp_obj_type_t pin_cpu_pins_obj_type = {
    { &mp_type_type },
    .name = MP_QSTR_cpu,
    .print = pin_named_pins_obj_print,
    .locals_dict = (mp_obj_t)&pin_cpu_pins_locals_dict,
};

const mp_obj_type_t pin_board_pins_obj_type = {
    { &mp_type_type },
    .name = MP_QSTR_board,
    .print = pin_named_pins_obj_print,
    .locals_dict = (mp_obj_t)&pin_board_pins_locals_dict,
};

const pin_obj_t *pin_find_named_pin(const mp_obj_dict_t *named_pins, mp_obj_t name) {
    mp_map_t *named_map = mp_obj_dict_get_map((mp_obj_t)named_pins);
    mp_map_elem_t *named_elem = mp_map_lookup(named_map, name, MP_MAP_LOOKUP);
    if (named_elem != NULL && named_elem->value != NULL) {
        return named_elem->value;
    }
    return NULL;
}

