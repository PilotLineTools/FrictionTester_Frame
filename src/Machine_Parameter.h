#ifndef MACHINE_PARAMETER_H
#define MACHINE_PARAMETER_H

#include <Arduino.h>
#include <algorithm>
// #include <EEPROM.h>
#include <Preferences.h>

/* macros for determining the max and min values of types */
#define issigned(t) (((t)(-1)) < ((t)0))

#define umaxof(t) (((0x1ULL << ((sizeof(t) * 8ULL) - 1ULL)) - 1ULL) | \
                   (0xFULL << ((sizeof(t) * 8ULL) - 4ULL)))

#define smaxof(t) (((0x1ULL << ((sizeof(t) * 8ULL) - 1ULL)) - 1ULL) | \
                   (0x7ULL << ((sizeof(t) * 8ULL) - 4ULL)))

#define maxof(t) ((unsigned long long)(issigned(t) ? smaxof(t) : umaxof(t)))
#define minof(t) ((long long)(issigned(t) ? -smaxof(t) : 0))

template <typename T>
class MachineParameter
{
private:
  uint16_t eepromAddr; // eeprom address of this parameter assinged during init function
  T defaultValue;      // assigned during construction
  T minValue;
  T maxValue;
  std::string name;       // Display name (human-readable)
  std::string storageKey; // NVS key (max 15 chars)

  // Generate NVS-compatible key from display name (max 15 chars)
  std::string generateKey(const char *displayName)
  {
    std::string key = displayName;
    // Remove spaces, parentheses, slashes
    key.erase(std::remove_if(key.begin(), key.end(),
                             [](char c)
                             { return c == ' ' || c == '(' || c == ')' || c == '/'; }),
              key.end());
    // Truncate to 15 chars (NVS limit)
    if (key.length() > 15)
    {
      key = key.substr(0, 15);
    }
    return key;
  }

public:
  T value; // this is where the parameter value is stored, it's type depends on the <typename> of the class

  MachineParameter() {}

  MachineParameter(T value)
  {
    defaultValue = value;
    minValue = minof(T);
    maxValue = maxof(T);
    this->value = defaultValue;
  }

  MachineParameter(const char *name, T value)
  {
    defaultValue = value;
    minValue = minof(T);
    maxValue = maxof(T);
    this->value = defaultValue;
    this->name = name;
    this->storageKey = generateKey(name);
  }

  MachineParameter(T value, T min, T max)
  {
    defaultValue = value;
    minValue = min;
    maxValue = max;
    this->value = defaultValue;
  }

  MachineParameter(const char *name, T value, T min, T max)
  {
    defaultValue = value;
    minValue = min;
    maxValue = max;
    this->value = defaultValue;
    this->name = name;
    this->storageKey = generateKey(name);
  }

  bool setValue(T value)
  {
    if (value > maxValue)
    {
      return 0;
    }
    else if (value < minValue)
    {
      return 0;
    }
    else
    {
      this->value = value;
      Preferences prefs;
      prefs.begin("params", false);

      if constexpr (std::is_same<T, float>::value)
      {
        prefs.putFloat(storageKey.c_str(), value);
      }
      else if constexpr (std::is_same<T, uint8_t>::value)
      {
        prefs.putUChar(storageKey.c_str(), value);
      }
      else if constexpr (std::is_same<T, uint16_t>::value)
      {
        prefs.putUShort(storageKey.c_str(), value);
      }
      else if constexpr (std::is_same<T, uint32_t>::value)
      {
        prefs.putUInt(storageKey.c_str(), value);
      }

      prefs.end();
      return 1;
    }
  }

  bool increaseValue(T value)
  {
    T tempValue = this->value + value;
    return setValue(tempValue);
  }

  bool decreaseValue(T value)
  {
    T tempValue = this->value - value;
    return setValue(tempValue);
  }

  T getValue()
  { // returns the value
    return value;
  }

  T getMin()
  { // returns the minimum value
    return minValue;
  }

  T getMax()
  { // returns the maximum value
    return maxValue;
  }

  const char *getName()
  {
    return name.c_str();
  }

  T getDefault()
  { // returns the default value
    return defaultValue;
  }

  void init(void)
  { // takes eeprom address of parameter and returns next available address based on size of parameter
    Preferences prefs;
    const char *ns = "params"; // Namespace for NVS
    int32_t getdata = 0;
    // Start NVS in read-write mode
    if (!prefs.begin(ns, false))
    {
      USBSerial.println("ERROR: Failed to open NVS namespace!");
      // Return 0 or some error code if needed
      // return 0;
    }

    if constexpr (std::is_same<T, float>::value)
    {
      if (prefs.isKey(storageKey.c_str()))
      {
        value = prefs.getFloat(storageKey.c_str(), defaultValue);
        // USBSerial.print("Retrieved float value for '");
        // USBSerial.print(name.c_str());
        // USBSerial.print("' (key: ");
        // USBSerial.print(storageKey.c_str());
        // USBSerial.print("): ");
        // USBSerial.println(value);
      }
      else
      {
        USBSerial.print("Creating NVS entry for '");
        USBSerial.print(name.c_str());
        USBSerial.print("' (key: ");
        USBSerial.print(storageKey.c_str());
        USBSerial.println(")");
        prefs.putFloat(storageKey.c_str(), defaultValue);
        value = defaultValue;
      }
    }

    else if constexpr (std::is_same<T, uint8_t>::value)
    {
      if (prefs.isKey(storageKey.c_str()))
      {
        value = prefs.getUChar(storageKey.c_str(), defaultValue);
        // USBSerial.print("Retrieved uint8_t value for '");
        // USBSerial.print(name.c_str());
        // USBSerial.print("' (key: ");
        // USBSerial.print(storageKey.c_str());
        // USBSerial.print("): ");
        // USBSerial.println(value);
      }
      else
      {
        prefs.putUChar(storageKey.c_str(), defaultValue);
        value = defaultValue;
      }
    }

    else if constexpr (std::is_same<T, uint16_t>::value)
    {
      if (prefs.isKey(storageKey.c_str()))
      {
        value = prefs.getUShort(storageKey.c_str(), defaultValue);
        // USBSerial.print("Retrieved uint16_t value for '");
        // USBSerial.print(name.c_str());
        // USBSerial.print("' (key: ");
        // USBSerial.print(storageKey.c_str());
        // USBSerial.print("): ");
        // USBSerial.println(value);
      }
      else
      {
        USBSerial.print("Creating NVS entry for '");
        USBSerial.print(name.c_str());
        USBSerial.print("' (key: ");
        USBSerial.print(storageKey.c_str());
        USBSerial.println(")");
        prefs.putUShort(storageKey.c_str(), defaultValue);
        value = defaultValue;
      }
    }

    else if constexpr (std::is_same<T, uint32_t>::value)
    {
      if (prefs.isKey(storageKey.c_str()))
      {
        value = prefs.getUInt(storageKey.c_str(), defaultValue);
        // USBSerial.print("Retrieved uint32_t value for '");
        // USBSerial.print(name.c_str());
        // USBSerial.print("' (key: ");
        // USBSerial.print(storageKey.c_str());
        // USBSerial.print("): ");
        // USBSerial.println(value);
      }
      else
      {
        prefs.putUInt(storageKey.c_str(), defaultValue);
        value = defaultValue;
      }
    }
    prefs.end();
  }

  void printValues()
  {
    USBSerial.print(name.c_str());
    USBSerial.print(" is set to ");
    USBSerial.println(value);
    USBSerial.print("Default = ");
    USBSerial.println(defaultValue);
    USBSerial.print("min = ");
    USBSerial.println(minValue);
    USBSerial.print("max = ");
    USBSerial.println(maxValue);
  }

  void setFromSerial()
  {
    printValues();
    USBSerial.println("Enter a new value (you have 10 seconds):");

    unsigned long startTime = millis();
    while ((millis() - startTime) < 10000)
    { // Wait up to 10 seconds
      if (USBSerial.available())
      {
        int newValue = USBSerial.parseInt(); // Read the integer value
        if (setValue(newValue))
        {
          USBSerial.print("Setting ");
          USBSerial.print(name.c_str());
          USBSerial.print(" to ");
          USBSerial.println(value);
          return; // Exit after processing input
        }
        else
        {
          USBSerial.println("Value out of range");
        }
      }
      delay(100); // Small delay to avoid busy-waiting
    }
    USBSerial.println("Time is up");
  }

  void saveToEEPROM()
  {
    // EEPROM.put(eepromAddr, value);
  }

  void getFromEEPROM()
  {
    // EEPROM.get(eepromAddr, value);
  }
};

/* "Explicit Specialization" of setFromSerial template for float types */
template <>
void MachineParameter<float>::setFromSerial()
{
  printValues();
  USBSerial.println("Enter a new value (you have 10 seconds):");

  unsigned long startTime = millis();
  while ((millis() - startTime) < 10000)
  { // Wait up to 10 seconds
    if (USBSerial.available())
    {
      int newValue = USBSerial.parseInt(); // Read the integer value
      if (setValue(newValue))
      {
        USBSerial.print("Setting ");
        USBSerial.print(name.c_str());
        USBSerial.print(" to ");
        USBSerial.println(value);
        return; // Exit after processing input
      }
      else
      {
        USBSerial.println("Value out of range");
      }
    }
    delay(100); // Small delay to avoid busy-waiting
  }
}

#endif
