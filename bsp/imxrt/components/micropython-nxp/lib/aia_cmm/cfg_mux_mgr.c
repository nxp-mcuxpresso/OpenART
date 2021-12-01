#include <stdint.h>
#include <stdio.h>
#include "board.h"
#include "cfg_mux_mgr.h"
#define _CMM_PoolSize (256 - 8)
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

typedef struct _CMM_Main_t
{
    mp_obj_dict_t *pDict;	
}CMM_State_t, CMM_Obj_t;

CMM_Obj_t s_cmm;

STATIC mp_obj_t CMM_Init(void) {
    if (s_cmm.pDict == 0) {
    }
    return mp_const_none;
}

mp_obj_t CMM_Deinit(void) {
	if (s_cmm.pDict) {
		mp_state_ctx.vm.pvPortRoots[0] = 0;
		gc_free(s_cmm.pDict);
		s_cmm.pDict = 0;
	}
    return mp_const_none;
}

static mp_obj_t _prvMux_Query(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData, 
    bool isTake, bool isPreempt)
{
	char szCombo[CMM_COMBOKEY_CAP]; /* must <= the size of pMuxData->szComboKey */
    mp_obj_t objRet = mp_const_none;;
	
	
	
	if (unit < 0) /* if a function does not have sub units, then unit < 0 */
	{
		snprintf(szCombo, sizeof(szCombo), "%s.%s", pszFn, pszSignal);
	} else {
        snprintf(szCombo, sizeof(szCombo), "%s.%d.%s", pszFn, unit, pszSignal);
    }
    if (pMuxData) {
        memset(pMuxData, 0, sizeof(*pMuxData));
        strcpy(pMuxData->szComboKey, szCombo);
    }
    else
    {
        return mp_const_none;
    }
	
	if (s_cmm.pDict == NULL)
	{
		pMuxData->pPinObj = mp_const_none;
		return mp_const_none;
	}
	
    //rocky: do not use qstr nor MP_DEFINE_STR_OBJ!
    mp_obj_str_t *pStrComboKey = mp_obj_new_str(szCombo, strlen(szCombo));
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0)/*try*/ {
        mp_obj_tuple_t *pTup = (mp_obj_tuple_t*) mp_obj_dict_get(s_cmm.pDict, (mp_obj_t)pStrComboKey);
        if (!pMuxData) /* If this is called from Python, we ju st return the tuple data */
	    {
		    nlr_pop();
		    goto cleanup;
	    }
	    /* We got the pin information as a tuple: (hint str, pin str, pin object, owner object) */
        mp_obj_t objHint, objPinObj, objOwner;
        objHint = pTup->items[0];
        objPinObj = pTup->items[2];
        objOwner = pTup->items[3];
        pMuxData->pPinObj = objPinObj;
        GET_STR_DATA_LEN(objHint, s, l);
        snprintf(pMuxData->szHint, sizeof(pMuxData->szHint), "%s", s);
        if (isTake) {
            if (isPreempt || mp_obj_is_small_int(objOwner) || objOwner == mp_const_none || objOwner == userObj) {
                /* free pin object or preempt */
                pTup->items[3] = userObj; /* well, py tuples are immutable, but we are in C */
            } else {
                pMuxData->pPinObj = mp_const_none; // failed to take
            }
            
        }
        nlr_pop();
        objRet = (mp_obj_t)pTup;
    } else /*except*/{
        // not found
        objRet = mp_const_none;
    }
cleanup:
    return objRet;
}

int Mux_Give(MuxItem_t *pMuxData)
{
    /* well, py tuples are immutable, but we are in C, set back the old owner */
    /* tuple layout: (hint string, pin string, pin object, owner object) */
    assert(pMuxData);
  //  assert(pMuxData->pTuple);
    if (s_cmm.pDict == NULL)
	{
		return 0;
	}
	
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0)/*try*/ {
        mp_obj_str_t *pStrComboKey = mp_obj_new_str(pMuxData->szComboKey, strlen(pMuxData->szComboKey));
        mp_obj_tuple_t *pTup = (mp_obj_tuple_t*) mp_obj_dict_get(s_cmm.pDict, pStrComboKey);
        pTup->items[3] = 0; // owner cleared
        nlr_pop();
    } else /*except*/{
        // not found
    }    
    return 1;
}

int Mux_Take(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData) 
{
    assert(pMuxData);
	  MuxItem_t *item;
     mp_obj_tuple_t *pTup = _prvMux_Query(userObj, pszFn, unit, pszSignal, pMuxData, TRUE, TRUE);
    return 1;
}

int Mux_TryTake(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData) 
{
    assert(pMuxData);
    mp_obj_t obj = _prvMux_Query(userObj, pszFn, unit, pszSignal, pMuxData, TRUE, FALSE);
    if (pMuxData->pPinObj == 0) 
        return -1L;
    return 1;
}

int Mux_Query(mp_obj_t userObj, const char *pszFn, int unit, const char *pszSignal, MuxItem_t *pMuxData) 
{
    assert(pMuxData);
    mp_obj_t obj = _prvMux_Query(userObj, pszFn, unit, pszSignal, pMuxData, FALSE, FALSE);
    return 1;
}

int Mux_GiveMany(MuxItem_t *pMuxData, int cnt) {
    for (int i=0; i<cnt; i++ , pMuxData++) {
        Mux_Give(pMuxData);
    }
    return 1;
}

int Mux_TakeMany(mp_obj_t userObj, const char *pszFn, int unit, const char **ppszSignals, int signalCnt, MuxItem_t *pMuxData) {
    int i;
    for (i=0; i<signalCnt; i++) {
        Mux_Take(userObj, pszFn, unit, ppszSignals[i], pMuxData + i);
    }
    return 1;
}

int Mux_TryTakeMany(mp_obj_t userObj, const char *pszFn, int unit, const char **ppszSignals, int signalCnt, MuxItem_t *pMuxData) {
    int i;
    int ret;
    for (i=0; i<signalCnt; i++) {
        ret = Mux_Take(userObj, pszFn, unit, ppszSignals[i], pMuxData + i);
        if (ret < 0) {
            // we failed one, give all taken
            while (i--) {
                Mux_Give(pMuxData + i);
            }
            break;
        }
    }
    return ret;
}

int Mux_QueryMany(mp_obj_t userObj, const char *pszFn, int unit, const char **ppszSignals, int signalCnt, MuxItem_t *pMuxData) {
    int i;
    for (i=0; i<signalCnt; i++) {
        Mux_Query(userObj, pszFn, unit, ppszSignals[i], pMuxData + i);
    }
    return 1;
}

mp_obj_t CMM_Query(mp_obj_t keyFn, mp_obj_t keyUnit, mp_obj_t keySignal) {
    const char *pszFn = mp_obj_str_get_str(keyFn);
    int unit = MP_OBJ_SMALL_INT_VALUE(keyUnit);
    const char *pszSignal = mp_obj_str_get_str(keySignal);
    MuxItem_t item;
    mp_obj_tuple_t *pTup = _prvMux_Query(mp_const_none, pszFn, unit, pszSignal, &item, FALSE, FALSE);
    return pTup;
}

/*static */mp_obj_t CMM_Add(mp_obj_t dict) {
    char szCombo[128] = "hw.-";
	s_cmm.pDict = dict;
	
	mp_obj_str_t *pStrComboKey = mp_obj_new_str(szCombo, strlen(szCombo));
	mp_obj_dict_t *self = MP_OBJ_TO_PTR(dict);
    mp_map_elem_t *elem = mp_map_lookup(&self->map, (mp_obj_t)pStrComboKey, MP_MAP_LOOKUP);
	if (elem == NULL)
	{
		s_cmm.pDict = NULL;
		mp_obj_t ret = MP_OBJ_NEW_SMALL_INT(0);
		mp_printf(&mp_plat_print, "Error Not find hw info in cmm_cfg.csv!!\r\n");
		return ret;
	}
	
	mp_obj_tuple_t *pTup = elem->value;
	mp_obj_t objHint, objPinObj, objOwner;
	objHint = pTup->items[0];
	objPinObj = pTup->items[1];
	mp_obj_str_t *pChipStr = mp_obj_new_str(CHIP_NAME,strlen(CHIP_NAME));
	mp_obj_str_t *pBoardStr = mp_obj_new_str(BOARD_NAME,strlen(BOARD_NAME));
	
	if ((mp_obj_str_equal(pChipStr,objHint) != true) || (mp_obj_str_equal(pBoardStr,objPinObj) != true)) 
	{
		GET_STR_DATA_LEN(objHint, hint_str, hint_len);
		GET_STR_DATA_LEN(objPinObj, obj_str, obj_len);
		
		s_cmm.pDict = NULL;
		mp_obj_t ret = MP_OBJ_NEW_SMALL_INT(0);
		mp_printf(&mp_plat_print, "Error cmm_cfg.csv hw (%.*s:%.*s) -- (%s:%s)mismatch!!\r\n",hint_len,hint_str,obj_len,obj_str,CHIP_NAME,BOARD_NAME);
		return ret;
	}
	
    mp_state_ctx.vm.pvPortRoots[0] = dict;
    mp_obj_t ret = MP_OBJ_NEW_SMALL_INT(1);
	return ret;
}

STATIC mp_map_elem_t *dict_iter_next(mp_obj_dict_t *dict, size_t *cur) {
    size_t max = dict->map.alloc;
    mp_map_t *map = &dict->map;

    for (size_t i = *cur; i < max; i++) {
        if (mp_map_slot_is_filled(map, i)) {
            *cur = i + 1;
            return &(map->table[i]);
        }
    }

    return NULL;
}

STATIC mp_obj_t CMM_Print() {
	const mp_print_t *print =  &mp_plat_print;
	mp_print_kind_t kind = MICROPY_PY_UJSON;
    mp_obj_dict_t *self = s_cmm.pDict;
    bool first = true;
	
	if (s_cmm.pDict == NULL)
	{
		return mp_const_none;
	}
	
    if (!(MICROPY_PY_UJSON && kind == PRINT_JSON)) {
        kind = PRINT_REPR;
    }
    if (MICROPY_PY_COLLECTIONS_ORDEREDDICT && self->base.type != &mp_type_dict && kind != PRINT_JSON) {
        mp_printf(print, "%q(", self->base.type->name);
    }
    mp_print_str(print, "{");
    size_t cur = 0;
    mp_map_elem_t *next = NULL;
    while ((next = dict_iter_next(self, &cur)) != NULL) {
        if (!first) {
            mp_print_str(print, ", ");
        }
        first = false;
        bool add_quote = MICROPY_PY_UJSON && kind == PRINT_JSON && !mp_obj_is_str_or_bytes(next->key);
        if (add_quote) {
            mp_print_str(print, "\"");
        }
        mp_obj_print_helper(print, next->key, kind);
        if (add_quote) {
            mp_print_str(print, "\"");
        }
        mp_print_str(print, ": ");
        mp_obj_print_helper(print, next->value, kind);
    }
    mp_print_str(print, "}");
    if (MICROPY_PY_COLLECTIONS_ORDEREDDICT && self->base.type != &mp_type_dict && kind != PRINT_JSON) {
        mp_print_str(print, ")");
    }
	return mp_const_none;
}


STATIC MP_DEFINE_CONST_FUN_OBJ_1(s_cmm_add_obj, CMM_Add);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(s_cmm_deinit_obj, CMM_Deinit);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(s_cmm_init_obj, CMM_Init);
STATIC MP_DEFINE_CONST_FUN_OBJ_3(s_cmm_find_obj, CMM_Query);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(s_cmm_print_obj, CMM_Print);

STATIC const mp_rom_map_elem_t s_cmm_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_add), MP_ROM_PTR(&s_cmm_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&s_cmm_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&s_cmm_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_find), MP_ROM_PTR(&s_cmm_find_obj) },
	{ MP_ROM_QSTR(MP_QSTR_print), MP_ROM_PTR(&s_cmm_print_obj) },
};
STATIC MP_DEFINE_CONST_DICT(s_cmm_locals_dict, s_cmm_locals_dict_table);

const mp_obj_module_t g_cmm_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&s_cmm_locals_dict,
};

