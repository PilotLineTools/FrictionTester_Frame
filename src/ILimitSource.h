#ifndef I_LIMIT_SOURCE_H
#define I_LIMIT_SOURCE_H

struct LimitSwitchState
{
   bool minTriggered = false;
   bool maxTriggered = false;
};

class ILimitSource
{
public:
   virtual ~ILimitSource() = default;
   // Return the latest externally-sourced limit state (for example from CAN).
   virtual LimitSwitchState readLimitState() const = 0;
};

#endif // I_LIMIT_SOURCE_H
