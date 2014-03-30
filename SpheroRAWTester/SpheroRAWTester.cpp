//######################################################################################################################
/*
    Copyright (c) since 2014 - Paul Freund

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/
//######################################################################################################################

#include "stdafx.h"
#include <windows.h>
#include "SpheroRAWItf.h"

#include <string>
#include <iostream>

using namespace std;

//======================================================================================================================

void PrintDeviceStatus(string action, ISpheroDevice* device) {
    cout << "Action: " << action << endl;

    if(device == nullptr) {
        cout << " |-- Error: Sphero handle is invalid" << endl;
        return;
    }

    switch(device->state()) {
        case SpheroState_None:                          { cout << " |-- SpheroRAW not initialized"                  << endl; break; }
        case SpheroState_Error_BluetoothError:          { cout << " |-- Error: Couldn't initialize Bluetooth stack" << endl; break; }
        case SpheroState_Error_BluetoothUnavailable:    { cout << " |-- Error: No valid Bluetooth adapter found"    << endl; break; }
        case SpheroState_Error_NotPaired:               { cout << " |-- Error: Specified Sphero not Paired"         << endl; break; }
        case SpheroState_Error_ConnectionFailed:        { cout << " |-- Error: Connecting failed"                   << endl; break; }
        case SpheroState_Disconnected:                  { cout << " |-- Sphero disconnected"                        << endl; break; }
        case SpheroState_Connected:                     { cout << " |-- Sphero connected"                           << endl; break; }
    }

    cout << endl;
}

//======================================================================================================================

int _tmain(int argc, _TCHAR* argv[])
{
    //------------------------------------------------------------------------------------------------------------------
    // Create device 
    ISpheroDevice* device = SpheroRAW_Create("Sphero-GRB");
    PrintDeviceStatus("SpheroRAW_Create(\"Sphero-GRB\");", device);

    //------------------------------------------------------------------------------------------------------------------
    // Connect 
    device->connect();
    PrintDeviceStatus("device->connect();", device);

    //------------------------------------------------------------------------------------------------------------------
    // Send/Receive Data
    for(; device->state() == SpheroState_Connected;) {
        device->receive();
        PrintDeviceStatus("device->receive();", device);
        Sleep(1000);
    }

    //------------------------------------------------------------------------------------------------------------------
    // Disconnect 
    device->disconnect();
    PrintDeviceStatus("device->disconnect();", device);

    //------------------------------------------------------------------------------------------------------------------
    // Destroy device 
    SpheroRAW_Destroy(device); device = nullptr;
    PrintDeviceStatus("SpheroRAW_Destroy(device); device = nullptr;", device);

    //------------------------------------------------------------------------------------------------------------------
    // Keep terminal open 
    cin.get();
	return 0;
}

//======================================================================================================================
