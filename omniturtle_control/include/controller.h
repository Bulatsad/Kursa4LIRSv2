#ifndef CONTROLLER_H
#define CONTROLLER_H

/// Controller interface class for various input devices

namespace OmniTurtle
{
 class InputController
 {
 public:
   virtual void getInput() =0;
   virtual void sendMessage() const =0;
 };
}

#endif // CONTROLLER_H
