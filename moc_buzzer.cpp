/****************************************************************************
** Meta object code from reading C++ file 'buzzer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../SAM_inProgress/src/peripherals/buzzer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'buzzer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_BuzzerWorker_t {
    QByteArrayData data[5];
    char stringdata0[58];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BuzzerWorker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BuzzerWorker_t qt_meta_stringdata_BuzzerWorker = {
    {
QT_MOC_LITERAL(0, 0, 12), // "BuzzerWorker"
QT_MOC_LITERAL(1, 13, 6), // "doWork"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 29), // "BuzzerConfig::buzzer_config_t"
QT_MOC_LITERAL(4, 51, 6) // "config"

    },
    "BuzzerWorker\0doWork\0\0BuzzerConfig::buzzer_config_t\0"
    "config"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BuzzerWorker[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void BuzzerWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BuzzerWorker *_t = static_cast<BuzzerWorker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->doWork((*reinterpret_cast< BuzzerConfig::buzzer_config_t(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< BuzzerConfig::buzzer_config_t >(); break;
            }
            break;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject BuzzerWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BuzzerWorker.data,
      qt_meta_data_BuzzerWorker,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *BuzzerWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BuzzerWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BuzzerWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int BuzzerWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
struct qt_meta_stringdata_Buzzer_t {
    QByteArrayData data[9];
    char stringdata0[99];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Buzzer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Buzzer_t qt_meta_stringdata_Buzzer = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Buzzer"
QT_MOC_LITERAL(1, 7, 4), // "buzz"
QT_MOC_LITERAL(2, 12, 0), // ""
QT_MOC_LITERAL(3, 13, 29), // "BuzzerConfig::buzzer_config_t"
QT_MOC_LITERAL(4, 43, 6), // "config"
QT_MOC_LITERAL(5, 50, 9), // "makeNoise"
QT_MOC_LITERAL(6, 60, 23), // "BuzzerConfig::BUZZ_TYPE"
QT_MOC_LITERAL(7, 84, 9), // "buzz_type"
QT_MOC_LITERAL(8, 94, 4) // "freq"

    },
    "Buzzer\0buzz\0\0BuzzerConfig::buzzer_config_t\0"
    "config\0makeNoise\0BuzzerConfig::BUZZ_TYPE\0"
    "buzz_type\0freq"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Buzzer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    2,   37,    2, 0x0a /* Public */,
       5,    1,   42,    2, 0x2a /* Public | MethodCloned */,
       5,    0,   45,    2, 0x2a /* Public | MethodCloned */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6, QMetaType::Int,    7,    8,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,

       0        // eod
};

void Buzzer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Buzzer *_t = static_cast<Buzzer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->buzz((*reinterpret_cast< BuzzerConfig::buzzer_config_t(*)>(_a[1]))); break;
        case 1: _t->makeNoise((*reinterpret_cast< BuzzerConfig::BUZZ_TYPE(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->makeNoise((*reinterpret_cast< BuzzerConfig::BUZZ_TYPE(*)>(_a[1]))); break;
        case 3: _t->makeNoise(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< BuzzerConfig::buzzer_config_t >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Buzzer::*)(BuzzerConfig::buzzer_config_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Buzzer::buzz)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Buzzer::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Buzzer.data,
      qt_meta_data_Buzzer,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Buzzer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Buzzer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Buzzer.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Buzzer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void Buzzer::buzz(BuzzerConfig::buzzer_config_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
