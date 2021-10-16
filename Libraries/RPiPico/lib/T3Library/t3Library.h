/** ----------------------------------------------------------------------------
Copyright 2020-2021 Tangibles That Teach, LLC, 4617 Chattahoochee Crossing SE,
Marietta, GA 30067, U.S.A. All rights reserved.

Tangibles That Teach, LLC has intellectual property rights relating to
technology embodied in this product. In particular, and without
limitation, these intellectual property rights may include one or more
of U.S. patents or pending patent applications in the U.S. and/or other countries.

This product is distributed under licenses restricting its use, copying
distribution, and decompilation. No part of this product may be
reproduced in any form by any means without prior written authorization
of Tangibles That Teach, LLC.

Contact Info:
Email: contact@tangiblesthatteach.com

@brief: Contains all outward facing library functions for Tangibles That Teach (T3) projects. These functions
 are sufficient to develop a main program with controller, sensors, and actuator that uses the T3 backend
 with timing and HTML5-based visual interface.
@author: Joel Dunham <joel.dunham@tangiblesthatteach.com>
@date: 2020/11/20
------------------------------------------------------------------------------ **/

#ifndef T3LIBRARY_H
#define T3LIBRARY_H

#include <stdint.h>
#if defined(SIMULATION) || defined(ARDUINO_ARCH_RP2040)
  #include <string>
  using myString = std::string;
#else
  #include <WString.h>
  using myString = String;
#endif

namespace T3Library {

// Forward declaration
namespace Generic {
  class T3InternalLibrary;
}

class T3Library {
  public:
    // Numbering matches messaging, so do not change without changing message definitions
    enum PROJECT_TYPE {
      GENERIC_PROJECT = 0,
      TRECS_PROJECT = 1,
      TRECS_ANALOG_PROJECT = 2,
      ROTATIONAL_PROJECT = 3,
      SOIL_LIQUIFACTION_PROJECT = 4
    };

    // Basic execution commands for limited direct control, if desired.
    enum EXECUTION_COMMANDS {
      STOP,
      CALIBRATE,
      SYSTEM_ID,
      RUN,
      WIFI_SETUP,
      SENSE_ONLY
    };

    /**
     * Constructor
     * @param: simulated (boolean) whether the hardware should be simulated (true) or whether the code should interact with the real hardware (false)
     */
    T3Library(bool simulated, PROJECT_TYPE type);

    /** 
     * Set the function call for a controller function 
     */
    void setControllerFunction(uint16_t (*controllerFuncPtr)(float goalValue, uint64_t time_ms, bool reset));

    // For handling the command
    /**
     * Defines the command
     */
    void defineCommand(float defaultValue, float minValue, float maxValue, myString name);

    /**
     * Gets the current command
     */
    float getCommand();

    /**
     * Sets the current command
     */
    void setCommand(float command);

    /**
     * Sets the version number. Default is 0.
     */
    void setVersion(uint8_t version);

    // For handling controller parameters
    /**
     * Obtain the parameter value by ID
     * @param id: uint8_t ID of the parameter
     * @param value: float pointer output value of the parameter
     * @return bool whether the value is valid
     */
    bool getParameterById(uint8_t id, float* value);

    /**
     * Set the parameter value by ID
     * @param id: uint8_t ID of the parameter
     * @param value: float value to set to the parameter
     * @return bool whether the value was set
     */
    bool setParameterById(uint8_t id, float value);

    /**
     * Add a parameter to the list of controller parameters managed through the front end
     * @param inDefaultValue: float the default value for the parameter
     * @param inHasMinValue: bool whether there is a minimum allowed value for the controller
     * @param inMinValue: float the minimum allowed value for the controller if inHasMinValue is true
     * @param inHasMaxValue: bool whether there is a maximum allowed value for the controller
     * @param inMaxValue: float the maximum allowed value for the controller if inHasMaxValue is true
     * @param inName myString name for the parameter to be displayed in the front end
     * @return int8_t: the returned ID of the parameter, -1 if the parameter was not added
     */
    int8_t addParameter(float inDefaultValue, bool inHasMinValue, float inMinValue,
                        bool inHasMaxValue, float inMaxValue, myString inName);

    // For handling sensors
    /**
     * Returns the sensor reading (after translation, if translation info is provided)
     * @param id: uint8_t ID of the sensor
     * @param value: float pointer value to be returned
     * @return bool whether the returned value is valid
     */
    bool getSensorById(uint8_t id, float* value);

    /**
     * Returns the filtered/estimated sensor reading (only different from getSensorById if there is a filter/estimator function defined)
     * @param id: uint8_t ID of the sensor
     * @param value: float pointer value to be returned
     * @return bool whether the returned value is valid
     */
    bool getFilteredSensorById(uint8_t id, float* value);
    /**
     * Add a sensor with an analog pin definition for automatically reading the sensor
     * @param pin: uint8_t pin value
     * @param hasTranslation: bool whether a linear translation is defined for obtaining a different representation
     * @param inMinTranslation: float translated value that corresponds to the lower bound of the analog value, if hasTranslation == true
     * @param inMaxTranslation: float translated value that corresponds to the upper bound of the analog value, if hasTranslation == true
     * @param hasFilter: bool whether there is a filter function
     * @param filterFuncPtr: pointer to the filter function for this sensor
     * @param inName myString name for the sensor to be displayed in the front end
     * @return int8_t: the returned ID of the sensor, -1 if the sensor was not added
     */
    int8_t addSensor(uint8_t pin, bool hasTranslation, float inMinTranslation, float inMaxTranslation, bool hasFilter,
                     float (*filterFuncPtr)(float sensedValue, uint64_t time_ms, bool reset), myString inName);

    /**
     * Add a sensor with a user-defined function pointer for reading the sensor (if it is not an analog read).
     * @param readFuncPtr: function pointer for reading the sensor value
     * @param hasTranslation: bool whether a linear translation is defined for obtaining a different representation
     * @param inMinTranslation: float translated value that corresponds to the lower bound of the analog value, if hasTranslation == true
     * @param inMaxTranslation: float translated value that corresponds to the upper bound of the analog value, if hasTranslation == true
     * @param hasFilter: bool whether there is a filter function
     * @param filterFuncPtr: pointer to the filter function for this sensor
     * @param inName myString name for the sensor to be displayed in the front end
     * @return int8_t: the returned ID of the sensor, -1 if the sensor was not added
     */
    int8_t addSensorWithReadFunction(float (*readFuncPtr)(uint8_t sensorId), bool hasTranslation, float inMinTranslation, float inMaxTranslation, bool hasFilter,
                                     float (*filterFuncPtr)(float sensedValue, uint64_t time_ms, bool reset), myString inName);

    /**
     * Enables the user to define a linear model for the sensor
     * @param id: uint8_t the sensor ID
     * @param zeroIntercept: float value of the zero intercept
     * @param slope: float value of the slope
     * @return bool: whether the model was set
     */
    bool setSensorLinearModel(uint8_t id, float zeroIntercept, float slope);

    /**
     * Set to print debug messages
     */
    void enableDebug();

    /**
     * Sets the local UDP server port (if in use) and the service name (e.g. trecs or unique project name)
     */
    void setLocalPortAndService(uint16_t port, myString service_name);

    /**
     * Overrides the default speed control pin. Must be set before setup is run.
     * @param pin: int8_t the output motor control pin. Must be PWM-output capable.
     */
    void setMotorControlPin(int8_t pin);

    /**
     * Enables setting a frequency through the main file, for cases
     *  where the default is not valid for the given project
     *
     * @param frequency: float the update frequency of the system
     */
    void setUpdateFrequency(float frequency);

    // The initial setup function for the T3 library
    void setup();

    // The update function for the T3 library
    void update();

    // Enables some commands to run directly from this library, if interation is available.
    void executeCommand(EXECUTION_COMMANDS command);

  private:
    Generic::T3InternalLibrary* __internalLibraryPtr;  // Since forward declared, can only be a pointer here
};

}  // namespace T3Library

#endif
