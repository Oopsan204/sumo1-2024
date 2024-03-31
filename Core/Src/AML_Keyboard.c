/*
  ADKeyboard 1.0 library 

  Last updated: 10PM 13/06/2023 UTC+7
  Authors: anmh1205

*/

#include <AML_Keyboard.h>


// ADKeyboard Module
int adc_key_val[5] = {30, 70, 110, 150, 600};
int NUM_KEYS = 5;
int adc_key_in;
int key = -1;
int oldkey = -1;
int keyValue; // lưu nút đã bấm

void (*resetFunc)(void) = 0; // declare reset function at address 0


// Convert ADC value to key number
int AML_Keyboard_getKey(int inputValue)
{
  int k;
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (inputValue < adc_key_val[k])
    {
      return k;
    }
  }
  if (k >= NUM_KEYS)
    k = -1; // No valid key pressed
  return k;
}





// Đọc nút cho đến khi phát hiện bấm nút
int AML_Keyboard_readKeyLoop()
{
  boolean flagPressButton = false;
  adc_key_in = analogRead(0); // read the value from the sensor pin A0
  key = AML_Keyboard_getKey(adc_key_in);  // convert into key press

  if (key != oldkey) // if keypress is detected
  {
    delay(50);                  // wait for debounce time
    adc_key_in = analogRead(0); // read the value from the sensor
    key = AML_Keyboard_getKey(adc_key_in);  // convert into key press
    if (key != oldkey)
    {
      oldkey = key;
      if (key >= 0)
      {
        switch (key)
        {
        case 0:
          keyValue = 0;
          break;
        case 1:
          keyValue = 1;
          break;
        case 2:
          keyValue = 2;
          break;
        case 3:
          keyValue = 3;
          break;
        case 4:
          keyValue = 4;
          break;
        }

        flagPressButton = true;
      }
    }
  }

  if (!flagPressButton) // chưa bấm
  {
    AML_Keyboard_readKeyLoop();
  }
  else
  {
    return keyValue; // thoát hàm
  }
}

// Đọc nút reset
void AML_Keyboard_readResetKey()
{
  adc_key_in = analogRead(0); // read the value from the sensor pin A0
  key = AML_Keyboard_getKey(adc_key_in);  // convert into key press

  if (key != oldkey) // if keypress is detected
  {
    delay(25);                  // wait for debounce time
    adc_key_in = analogRead(0); // read the value from the sensor
    key = AML_Keyboard_getKey(adc_key_in);  // convert into key press
    if (key != oldkey)
    {
      oldkey = key;
      if (key >= 0)
      {
        switch (key)
        {
        case 0:
          keyValue = 0;
          break;
        case 1:
          keyValue = 1;
          break;
        case 2:
          keyValue = 2;
          break;
        case 3:
          keyValue = 3;
          break;
        case 4:
          keyValue = 4;
          break;
        }

        if (keyValue == 1) // Nút B
        {
          resetFunc(); // Lệnh reset
        }
      }
    }
  }
}