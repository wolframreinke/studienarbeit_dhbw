/****************************************************************************
** Meta object code from reading C++ file 'log_player.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "log_player.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'log_player.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LogPlayer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x05,
      21,   10,   10,   10, 0x05,
      43,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
      59,   10,   10,   10, 0x08,
      73,   10,   10,   10, 0x0a,
      84,   10,   10,   10, 0x0a,
      98,   10,   10,   10, 0x0a,
     111,   10,   10,   10, 0x0a,
     118,   10,   10,   10, 0x0a,
     129,   10,   10,   10, 0x0a,
     143,   10,   10,   10, 0x0a,
     160,   10,   10,   10, 0x0a,
     180,   10,   10,   10, 0x0a,
     192,   10,   10,   10, 0x0a,
     203,   10,   10,   10, 0x0a,
     216,   10,   10,   10, 0x0a,
     235,  229,   10,   10, 0x0a,
     256,  250,   10,   10, 0x0a,
     271,   10,   10,   10, 0x0a,
     282,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LogPlayer[] = {
    "LogPlayer\0\0updated()\0recoverTimerHandled()\0"
    "quitRequested()\0handleTimer()\0stepBack()\0"
    "stepForward()\0playOrStop()\0stop()\0"
    "playBack()\0playForward()\0accelerateBack()\0"
    "accelerateForward()\0goToFirst()\0"
    "goToLast()\0decelerate()\0accelerate()\0"
    "index\0goToIndex(int)\0cycle\0goToCycle(int)\0"
    "showLive()\0setLiveMode()\0"
};

void LogPlayer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LogPlayer *_t = static_cast<LogPlayer *>(_o);
        switch (_id) {
        case 0: _t->updated(); break;
        case 1: _t->recoverTimerHandled(); break;
        case 2: _t->quitRequested(); break;
        case 3: _t->handleTimer(); break;
        case 4: _t->stepBack(); break;
        case 5: _t->stepForward(); break;
        case 6: _t->playOrStop(); break;
        case 7: _t->stop(); break;
        case 8: _t->playBack(); break;
        case 9: _t->playForward(); break;
        case 10: _t->accelerateBack(); break;
        case 11: _t->accelerateForward(); break;
        case 12: _t->goToFirst(); break;
        case 13: _t->goToLast(); break;
        case 14: _t->decelerate(); break;
        case 15: _t->accelerate(); break;
        case 16: _t->goToIndex((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->goToCycle((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 18: _t->showLive(); break;
        case 19: _t->setLiveMode(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LogPlayer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LogPlayer::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_LogPlayer,
      qt_meta_data_LogPlayer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LogPlayer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LogPlayer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LogPlayer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LogPlayer))
        return static_cast<void*>(const_cast< LogPlayer*>(this));
    return QObject::qt_metacast(_clname);
}

int LogPlayer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    }
    return _id;
}

// SIGNAL 0
void LogPlayer::updated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void LogPlayer::recoverTimerHandled()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void LogPlayer::quitRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}
QT_END_MOC_NAMESPACE
