#ifndef sbpl_adim_macros_h
#define sbpl_adim_macros_h

#include <memory>

#define SBPL_CLASS_FORWARD(classname) \
class classname;\
typedef std::shared_ptr<classname> classname##Ptr;\
typedef std::shared_ptr<const classname> classname##ConstPtr;

#endif
