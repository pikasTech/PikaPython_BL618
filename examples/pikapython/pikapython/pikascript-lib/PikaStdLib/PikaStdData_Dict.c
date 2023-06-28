#include "PikaStdData_Dict.h"
#include "BaseObj.h"
#include "PikaObj.h"
#include "PikaStdData_Tuple.h"
#include "PikaStdData_dict_items.h"
#include "PikaStdData_dict_keys.h"
#include "PikaStdLib_SysObj.h"
#include "dataStrs.h"

Arg* PikaStdData_Dict_get(PikaObj* self, char* key) {
    return arg_copy(objDict_get(self, key));
}

void PikaStdData_Dict___init__(PikaObj* self) {
    objDict_init(self);
}

void PikaStdData_Dict_set(PikaObj* self, char* key, Arg* arg) {
    objDict_set(self, key, arg);
}

void PikaStdData_Dict_remove(PikaObj* self, char* key) {
    PikaDict* dict = obj_getPtr(self, "dict");
    PikaDict* keys = obj_getPtr(self, "_keys");
    pikaDict_removeArg(dict, pikaDict_getArg(dict, key));
    pikaDict_removeArg(keys, pikaDict_getArg(keys, key));
}

Arg* PikaStdData_Dict___iter__(PikaObj* self) {
    obj_setInt(self, "__iter_i", 0);
    return arg_newRef(self);
}

Arg* PikaStdData_Dict___next__(PikaObj* self) {
    int __iter_i = args_getInt(self->list, "__iter_i");
    PikaDict* keys = obj_getPtr(self, "_keys");
    Arg* res = arg_copy(args_getArgByIndex(&keys->super, __iter_i));
    if (NULL == res) {
        return arg_newNone();
    }
    args_setInt(self->list, "__iter_i", __iter_i + 1);
    return res;
}

void PikaStdData_Dict___setitem__(PikaObj* self, Arg* __key, Arg* __val) {
    PikaStdData_Dict_set(self, obj_getStr(self, "__key"),
                         obj_getArg(self, "__val"));
}

Arg* PikaStdData_Dict___getitem__(PikaObj* self, Arg* __key) {
    return PikaStdData_Dict_get(self, obj_getStr(self, "__key"));
}

void PikaStdData_Dict___del__(PikaObj* self) {
    PikaDict* dict = obj_getPtr(self, "dict");
    PikaDict* keys = obj_getPtr(self, "_keys");
    pikaDict_deinit(dict);
    if (NULL != keys) {
        pikaDict_deinit(keys);
    }
}

void PikaStdData_dict_keys___init__(PikaObj* self, PikaObj* dict) {
    obj_setPtr(self, "dictptr", dict);
}

PikaObj* PikaStdData_Dict_keys(PikaObj* self) {
    PikaObj* dict_keys = newNormalObj(New_PikaStdData_dict_keys);
    PikaStdData_dict_keys___init__(dict_keys, self);
    return dict_keys;
}

PikaObj* PikaStdData_Dict_items(PikaObj* self) {
    PikaObj* dict_items = newNormalObj(New_PikaStdData_dict_items);
    PikaStdData_dict_keys___init__(dict_items, self);
    return dict_items;
}

Arg* PikaStdData_dict_keys___iter__(PikaObj* self) {
    obj_setInt(self, "__iter_i", 0);
    return arg_newRef(self);
}

Arg* PikaStdData_dict_keys___next__(PikaObj* self) {
    int __iter_i = args_getInt(self->list, "__iter_i");
    PikaObj* dictptr = obj_getPtr(self, "dictptr");
    PikaDict* keys = obj_getPtr(dictptr, "_keys");
    Arg* res = arg_copy(args_getArgByIndex(&keys->super, __iter_i));
    if (NULL == res) {
        return arg_newNone();
    }
    args_setInt(self->list, "__iter_i", __iter_i + 1);
    return res;
}

char* builtins_str(PikaObj* self, Arg* arg);
char* PikaStdData_dict_keys___str__(PikaObj* self) {
    Arg* str_arg = arg_newStr("dict_keys([");
    PikaObj* dictptr = obj_getPtr(self, "dictptr");
    PikaDict* keys = obj_getPtr(dictptr, "_keys");

    int i = 0;
    while (PIKA_TRUE) {
        Arg* item = args_getArgByIndex(&keys->super, i);
        if (NULL == item) {
            break;
        }
        if (i != 0) {
            str_arg = arg_strAppend(str_arg, ", ");
        }
        char* item_str = builtins_str(self, item);
        if (arg_getType(item) == ARG_TYPE_STRING) {
            str_arg = arg_strAppend(str_arg, "'");
        }
        str_arg = arg_strAppend(str_arg, item_str);
        if (arg_getType(item) == ARG_TYPE_STRING) {
            str_arg = arg_strAppend(str_arg, "'");
        }
        i++;
    }

    str_arg = arg_strAppend(str_arg, "])");
    obj_setStr(self, "_buf", arg_getStr(str_arg));
    arg_deinit(str_arg);
    return obj_getStr(self, "_buf");
}

typedef struct {
    Arg* buf;
    int isFirst;
} DictToStrContext;

int32_t dictToStrEachHandle(PikaObj* self,
                            Arg* keyEach,
                            Arg* valEach,
                            void* context) {
    DictToStrContext* ctx = (DictToStrContext*)context;

    // 判断是否需要在字符串前添加逗号
    if (!ctx->isFirst) {
        ctx->buf = arg_strAppend(ctx->buf, ", ");
    } else {
        ctx->isFirst = 0;
    }

    // 将键值对添加到字符串
    ctx->buf = arg_strAppend(ctx->buf, "'");
    ctx->buf = arg_strAppend(ctx->buf, builtins_str(self, keyEach));
    ctx->buf = arg_strAppend(ctx->buf, "'");
    ctx->buf = arg_strAppend(ctx->buf, ": ");
    if (arg_getType(valEach) == ARG_TYPE_STRING) {
        ctx->buf = arg_strAppend(ctx->buf, "'");
    }
    ctx->buf = arg_strAppend(ctx->buf, builtins_str(self, valEach));
    if (arg_getType(valEach) == ARG_TYPE_STRING) {
        ctx->buf = arg_strAppend(ctx->buf, "'");
    }

    return 0;
}

char* PikaStdData_Dict___str__(PikaObj* self) {
    // 初始化上下文
    DictToStrContext context;
    context.buf = arg_newStr("{");
    context.isFirst = 1;

    // 使用 objDict_forEach 遍历字典
    objDict_forEach(self, dictToStrEachHandle, &context);

    // 将字符串添加到结果中并返回
    context.buf = arg_strAppend(context.buf, "}");
    obj_setStr(self, "_buf", arg_getStr(context.buf));
    arg_deinit(context.buf);
    return obj_getStr(self, "_buf");
}

int PikaStdData_Dict___len__(PikaObj* self) {
    PikaDict* dict = obj_getPtr(self, "dict");
    return args_getSize(&dict->super);
}

int PikaStdData_dict_keys___len__(PikaObj* self) {
    PikaObj* dictptr = obj_getPtr(self, "dictptr");
    PikaDict* keys = obj_getPtr(dictptr, "_keys");
    return args_getSize(&keys->super);
}

int dict_contains(PikaDict* dict, Arg* key) {
    int i = 0;
    while (PIKA_TRUE) {
        Arg* item = args_getArgByIndex(&dict->super, i);
        if (NULL == item) {
            break;
        }
        if (arg_isEqual(item, key)) {
            return PIKA_TRUE;
        }
        i++;
    }
    return pika_false;
}

int PikaStdData_Dict___contains__(PikaObj* self, Arg* val) {
    PikaDict* dict = obj_getPtr(self, "_keys");
    return dict_contains(dict, val);
}

Arg* PikaStdData_dict_items___iter__(PikaObj* self) {
    obj_setInt(self, "__iter_i", 0);
    return arg_newRef(self);
}

int PikaStdData_dict_items___len__(PikaObj* self) {
    PikaObj* dictptr = obj_getPtr(self, "dictptr");
    PikaDict* keys = obj_getPtr(dictptr, "_keys");
    return args_getSize(&keys->super);
}

Arg* PikaStdData_dict_items___next__(PikaObj* self) {
    int __iter_i = args_getInt(self->list, "__iter_i");
    PikaObj* dictptr = obj_getPtr(self, "dictptr");
    PikaDict* keys = obj_getPtr(dictptr, "_keys");
    PikaDict* dict = obj_getPtr(dictptr, "dict");
    Arg* key = args_getArgByIndex(&keys->super, __iter_i);
    Arg* val = args_getArgByIndex(&dict->super, __iter_i);
    if (NULL == key) {
        return arg_newNone();
    }
    PikaObj* tuple = newNormalObj(New_PikaStdData_Tuple);
    PikaStdData_Tuple___init__(tuple);
    PikaList* list = obj_getPtr(tuple, "list");
    pikaList_append(list, key);
    pikaList_append(list, val);
    args_setInt(self->list, "__iter_i", __iter_i + 1);
    return arg_newObj(tuple);
}

char* PikaStdData_dict_items___str__(PikaObj* self) {
    Arg* str_arg = arg_newStr("dict_items([");
    int i = 0;
    obj_setInt(self, "__iter_i", 0);
    while (PIKA_TRUE) {
        Arg* item = PikaStdData_dict_items___next__(self);
        if (arg_getType(item) == ARG_TYPE_NONE) {
            arg_deinit(item);
            break;
        }
        if (i != 0) {
            str_arg = arg_strAppend(str_arg, ", ");
        }
        char* item_str = builtins_str(self, item);
        str_arg = arg_strAppend(str_arg, item_str);
        i++;
        arg_deinit(item);
    }
    str_arg = arg_strAppend(str_arg, "])");
    obj_setStr(self, "_buf", arg_getStr(str_arg));
    arg_deinit(str_arg);
    return obj_getStr(self, "_buf");
}

void PikaStdData_Dict_update(PikaObj* self, PikaObj* other) {
    PikaObj* context = newNormalObj(New_PikaStdLib_SysObj);
    obj_setRef(context, "@other", other);
    obj_setRef(context, "@self", self);
    /* clang-format off */
    PIKA_PYTHON(
    for @item in @other:
        @self[@item] = @other[@item]
    
    )
    /* clang-format on */
    const uint8_t
        bytes[] =
            {
                0x40, 0x00, 0x00, 0x00, /* instruct array size */
                0x10, 0x81, 0x01, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x04,
                0x0d, 0x00, 0x00, 0x82, 0x11, 0x00, 0x00, 0x04, 0x1e, 0x00,
                0x00, 0x0d, 0x1e, 0x00, 0x00, 0x07, 0x24, 0x00, 0x11, 0x81,
                0x26, 0x00, 0x11, 0x01, 0x1e, 0x00, 0x21, 0x01, 0x01, 0x00,
                0x21, 0x01, 0x1e, 0x00, 0x11, 0x1d, 0x00, 0x00, 0x01, 0x02,
                0x2c, 0x00, 0x01, 0x04, 0x26, 0x00, 0x00, 0x86, 0x38, 0x00,
                0x00, 0x8c, 0x0d, 0x00, /* instruct array */
                0x3b, 0x00, 0x00, 0x00, /* const pool size */
                0x00, 0x40, 0x6f, 0x74, 0x68, 0x65, 0x72, 0x00, 0x69, 0x74,
                0x65, 0x72, 0x00, 0x24, 0x6c, 0x30, 0x00, 0x24, 0x6c, 0x30,
                0x2e, 0x5f, 0x5f, 0x6e, 0x65, 0x78, 0x74, 0x5f, 0x5f, 0x00,
                0x40, 0x69, 0x74, 0x65, 0x6d, 0x00, 0x32, 0x00, 0x40, 0x73,
                0x65, 0x6c, 0x66, 0x00, 0x5f, 0x5f, 0x73, 0x65, 0x74, 0x69,
                0x74, 0x65, 0x6d, 0x5f, 0x5f, 0x00, 0x2d, 0x31, 0x00, /* const
                                                                         pool */
            };
    pikaVM_runByteCode(context, (uint8_t*)bytes);
    obj_deinit(context);
}
