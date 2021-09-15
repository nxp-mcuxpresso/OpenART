/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * WINC1500 Python module.
 *
 */
#include <string.h>
#include <stdarg.h>
#include <errno.h>

#include "py/nlr.h"
#include "py/objtuple.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/runtime.h"
#include "lib/netutils/netutils.h"
#include "modnetwork.h"
#if MICROPY_PY_WLAN
#include "common.h"
#include "py_helper.h"
#include "ff_wrapper.h"

#include "winc.h"
#include <sys/socket.h>
typedef struct _winc_obj_t {
    mp_obj_base_t base;
	rt_wlan_mode_t wlan_mode;
} winc_obj_t;


const mp_obj_type_t mod_network_nic_type_winc;
// Initialise the module using the given SPI bus and pins and return a winc object.
static mp_obj_t py_winc_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *all_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = WINC_MODE_STA } },
    };
	
	winc_obj_t *self = m_new_obj(winc_obj_t);
	self = gc_alloc(sizeof(winc_obj_t), GC_ALLOC_FLAG_HAS_FINALISER);
    // parse args
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, all_args, &kw_args, MP_ARRAY_SIZE(args), allowed_args, args);

    // Init WINC
    winc_mode_t winc_mode = args[0].u_int;
    

    switch (winc_mode) {
        case WINC_MODE_BSP:
            printf("Running in BSP mode...\n");
            break;
        case WINC_MODE_FIRMWARE:
            printf("Running in Firmware Upgrade mode...\n");
            break;
        case WINC_MODE_AP:
			self->wlan_mode = RT_WLAN_AP;
            break;
        case WINC_MODE_STA:
            self->wlan_mode = RT_WLAN_STATION;
            break;
        default:
            nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "WiFi mode is not supported!"));
    }
	
	if (rt_wlan_set_mode("8266",self->wlan_mode) != RT_EOK) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "failed to init WINC1500 module"));
    }
	
	self->base.type  = &mod_network_nic_type_winc;
	
    return self;
}

// method connect(ssid, key=None, *, security=WPA2, bssid=None)
static mp_obj_t py_winc_connect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_key, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = WINC_SEC_WPA_PSK} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get ssid
    const char *ssid = mp_obj_str_get_str(args[0].u_obj);

    // get key and sec
    const char *key = NULL;
    mp_uint_t security = WINC_SEC_OPEN;

    if (args[1].u_obj != mp_const_none) {
        key = mp_obj_str_get_str(args[1].u_obj);
        security = args[2].u_int;
    }

    // connect to AP
    if (rt_wlan_connect(ssid, key) != RT_EOK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_RuntimeError,
                    "could not connect to ssid=%s, sec=%d, key=%s\n", ssid, security, key));
    }

    return mp_const_none;
}

// method start_ap(ssid, key=None, security=OPEN)
static mp_obj_t py_winc_start_ap(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid,     MP_ARG_REQUIRED| MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_key,      MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = WINC_SEC_OPEN } }, //M2M_WIFI_SEC_WPA_PSK
        { MP_QSTR_channel,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get ssid
    const char *ssid = mp_obj_str_get_str(args[0].u_obj);

    // get key and security
    const char *key = NULL;
    mp_uint_t security = args[2].u_int;

    

    key = mp_obj_str_get_str(args[1].u_obj);

    // get channel
    mp_uint_t channel = args[3].u_int;

    // Initialize WiFi in AP mode.
    if (rt_wlan_start_ap(ssid, key) != RT_EOK) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "failed to start in AP mode"));
    }

    printf("AP mode started. You can connect to %s.\r\n", ssid);
    return mp_const_none;
}

static mp_obj_t py_winc_disconnect(mp_obj_t self_in)
{
    rt_wlan_disconnect();
    return mp_const_none;
}

static mp_obj_t py_winc_isconnected(mp_obj_t self_in)
{
    return mp_obj_new_bool(rt_wlan_is_connected());
}

static mp_obj_t py_winc_connected_sta(mp_obj_t self_in)
{

    return mp_const_none;
}

static mp_obj_t py_winc_wait_for_sta(mp_obj_t self_in, mp_obj_t timeout_in)
{
    
    return mp_const_none;

}

static mp_obj_t py_winc_ifconfig(mp_obj_t self_in)
{
    uint8_t ip_addr[6];
    mp_obj_t ifconfig_list= mp_obj_new_list(0, NULL);
	struct rt_wlan_info info;
	
	int rssi = rt_wlan_get_rssi();
	rt_wlan_get_mac(ip_addr);
	rt_wlan_get_info(&info);
	
	VSTR_FIXED(ip_vstr, 16);
    vstr_printf(&ip_vstr, "%d.%d.%d.%d", ip_addr[0],
            ip_addr[1], ip_addr[2], ip_addr[3]);
	
	mp_obj_list_append(ifconfig_list, mp_obj_new_int(info.rssi));
	mp_obj_list_append(ifconfig_list, mp_obj_new_str((const char*)info.ssid.val, info.ssid.len));
    mp_obj_list_append(ifconfig_list, mp_obj_new_str(ip_vstr.buf, ip_vstr.len));
	
    return ifconfig_list;
}
STATIC mp_obj_t *wlan_scan_list = NULL;
static void wlan_station_scan(void)
{
    if (wlan_scan_list == NULL) {
        // called unexpectedly
        return;
    }
    
    struct rt_wlan_scan_result *scan_result = RT_NULL;

    /* scan ap info */
    scan_result = rt_wlan_scan_sync();
    if (scan_result)
    {
        int index, num;
        char *security;

        num = scan_result->num;
        for (index = 0; index < num; index ++)
        {
            switch (scan_result->info[index].security)
            {
            case SECURITY_OPEN:
                security = "OPEN";
                break;
            case SECURITY_WEP_PSK:
                security = "WEP_PSK";
                break;
            case SECURITY_WEP_SHARED:
                security = "WEP_SHARED";
                break;
            case SECURITY_WPA_TKIP_PSK:
                security = "WPA_TKIP_PSK";
                break;
            case SECURITY_WPA_AES_PSK:
                security = "WPA_AES_PSK";
                break;
            case SECURITY_WPA2_AES_PSK:
                security = "WPA2_AES_PSK";
                break;
            case SECURITY_WPA2_TKIP_PSK:
                security = "WPA2_TKIP_PSK";
                break;
            case SECURITY_WPA2_MIXED_PSK:
                security = "WPA2_MIXED_PSK";
                break;
            case SECURITY_WPS_OPEN:
                security = "WPS_OPEN";
                break;
            case SECURITY_WPS_SECURE:
                security = "WPS_SECURE";
                break;
            default:
                security = "UNKNOWN";
                break;
            }

            mp_obj_tuple_t *t = mp_obj_new_tuple(6, NULL);
            t->items[0] = mp_obj_new_bytes(&scan_result->info[index].ssid.val[0], strlen((char *)(&scan_result->info[index].ssid.val[0])));
            t->items[1] = mp_obj_new_bytes(&scan_result->info[index].bssid[0], strlen((char *)(&scan_result->info[index].bssid[0])));
            t->items[2] = MP_OBJ_NEW_SMALL_INT(scan_result->info[index].channel);
            t->items[3] = MP_OBJ_NEW_SMALL_INT(scan_result->info[index].rssi);
            t->items[4] = mp_obj_new_bytes((const byte *)security, strlen(security));
            t->items[5] = MP_OBJ_NEW_SMALL_INT(scan_result->info[index].hidden);
            
            mp_obj_list_append(*wlan_scan_list, MP_OBJ_FROM_PTR(t));

        }
        rt_wlan_scan_result_clean();
    }
    else
    {
        mp_printf(&mp_plat_print, ("wifi scan result is null\n"));
        *wlan_scan_list = MP_OBJ_NULL;
    }
}

static mp_obj_t py_winc_scan(mp_obj_t self_in)
{
    mp_obj_t list = mp_obj_new_list(0, NULL);
    wlan_scan_list = &list;
    wlan_station_scan();

    if (list == MP_OBJ_NULL) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "scan failed"));
    }
    return list;
}

static mp_obj_t py_winc_get_rssi(mp_obj_t self_in)
{
    return mp_obj_new_int(rt_wlan_get_rssi()); 
}

static mp_obj_t py_winc_fw_version(mp_obj_t self_in)
{

    return mp_const_none;
}

static mp_obj_t py_winc_fw_dump(mp_obj_t self_in, mp_obj_t path_in)
{
    

    return mp_const_none;
}

static mp_obj_t py_winc_fw_update(mp_obj_t self_in, mp_obj_t path_in)
{
    
    return mp_const_none;
}

static int py_winc_gethostbyname(mp_obj_t nic, const char *name, mp_uint_t len, uint8_t *out_ip)
{
    return 0;
}


static MP_DEFINE_CONST_FUN_OBJ_KW(py_winc_connect_obj, 1,   py_winc_connect);
static MP_DEFINE_CONST_FUN_OBJ_KW(py_winc_start_ap_obj,1,   py_winc_start_ap);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_disconnect_obj,    py_winc_disconnect);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_isconnected_obj,   py_winc_isconnected);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_connected_sta_obj, py_winc_connected_sta);
static MP_DEFINE_CONST_FUN_OBJ_2(py_winc_wait_for_sta_obj,  py_winc_wait_for_sta);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_ifconfig_obj,      py_winc_ifconfig);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_scan_obj,          py_winc_scan);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_get_rssi_obj,      py_winc_get_rssi);
static MP_DEFINE_CONST_FUN_OBJ_1(py_winc_fw_version_obj,    py_winc_fw_version);
static MP_DEFINE_CONST_FUN_OBJ_2(py_winc_fw_dump_obj,       py_winc_fw_dump);
static MP_DEFINE_CONST_FUN_OBJ_2(py_winc_fw_update_obj,     py_winc_fw_update);

static const mp_rom_map_elem_t winc_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_connect),       (mp_obj_t)&py_winc_connect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_start_ap),      (mp_obj_t)&py_winc_start_ap_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_disconnect),    (mp_obj_t)&py_winc_disconnect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_isconnected),   (mp_obj_t)&py_winc_isconnected_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_connected_sta), (mp_obj_t)&py_winc_connected_sta_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_wait_for_sta),  (mp_obj_t)&py_winc_wait_for_sta_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ifconfig),      (mp_obj_t)&py_winc_ifconfig_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_scan),          (mp_obj_t)&py_winc_scan_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_rssi),          (mp_obj_t)&py_winc_get_rssi_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_fw_version),    (mp_obj_t)&py_winc_fw_version_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_fw_dump),       (mp_obj_t)&py_winc_fw_dump_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_fw_update),     (mp_obj_t)&py_winc_fw_update_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_OPEN),            MP_OBJ_NEW_SMALL_INT(WINC_SEC_OPEN) },      // Network is not secured.
    { MP_OBJ_NEW_QSTR(MP_QSTR_WEP),             MP_OBJ_NEW_SMALL_INT(WINC_SEC_WEP) },       // Security type WEP (40 or 104) OPEN OR SHARED.
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA_PSK),       	MP_OBJ_NEW_SMALL_INT(WINC_SEC_WPA_PSK) },// Network secured with WPA/WPA2 personal(PSK).
    { MP_OBJ_NEW_QSTR(MP_QSTR_802_1X),          MP_OBJ_NEW_SMALL_INT(WINC_SEC_802_1X) },    // Network is secured with WPA/WPA2 Enterprise.
    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_STA),        MP_OBJ_NEW_SMALL_INT(WINC_MODE_STA) },          // Start in Staion mode.
    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_AP),         MP_OBJ_NEW_SMALL_INT(WINC_MODE_AP) },           // Start in Access Point mode.
    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_P2P),        MP_OBJ_NEW_SMALL_INT(WINC_MODE_P2P) },          // Start in P2P (WiFi Direct) mode.
    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_BSP),        MP_OBJ_NEW_SMALL_INT(WINC_MODE_BSP) },          // Init BSP.
    { MP_OBJ_NEW_QSTR(MP_QSTR_MODE_FIRMWARE),   MP_OBJ_NEW_SMALL_INT(WINC_MODE_FIRMWARE) },     // Start in Firmware Upgrade mode.
};

static MP_DEFINE_CONST_DICT(winc_locals_dict, winc_locals_dict_table);

const mp_obj_type_t mod_network_nic_type_winc = {
    { &mp_type_type },
    .name = MP_QSTR_WINC,
    .make_new = py_winc_make_new,
    .locals_dict = (mp_obj_dict_t *) &winc_locals_dict,

};
#endif