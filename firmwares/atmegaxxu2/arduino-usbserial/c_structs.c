
#include <LUFA/Version.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/USB/USB.h>

#include <LUFA/Common/Common.h>
#include <Descriptors.h>
#include <c_structs.h>

/** LUFA CDC Class driver interface configuration and state information. This 
 * structure is passed to all CDC Class driver functions, so that multiple 
 * instances of the same class within a device can be differentiated from one 
 * another. 
 */

USB_ClassInfo_MIDI_Device_t USB_MIDI_Interface =
    {   
        .Config =
            {   
                .StreamingInterfaceNumber = INTERFACE_ID_AudioStream,
                .DataINEndpoint           =   
                    {   
                        .Address          = MIDI_STREAM_IN_EPADDR,
                        .Size             = MIDI_STREAM_EPSIZE,
                        .Banks            = 1,
                    },  
                .DataOUTEndpoint           =   
                    {   
                        .Address          = MIDI_STREAM_OUT_EPADDR,
                        .Size             = MIDI_STREAM_EPSIZE,
                        .Banks            = 1,
                    },  
            },  
    }; 

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface = { 
    .Config = 
        { 
            .ControlInterfaceNumber = INTERFACE_ID_CDC_CCI, 
            .DataINEndpoint = 
                { 
                    .Address = CDC_TX_EPADDR, 
                    .Size = CDC_TXRX_EPSIZE, 
                    .Type = EP_TYPE_CONTROL, 
                    .Banks = 1, 
                }, 
            .DataOUTEndpoint = 
                { 
                    .Address = CDC_RX_EPADDR, 
                    .Size = CDC_TXRX_EPSIZE, 
                    .Type = EP_TYPE_CONTROL, 
                    .Banks = 1, 
                }, 
            .NotificationEndpoint = 
                { 
                    .Address = CDC_NOTIFICATION_EPADDR, 
                    .Size = CDC_NOTIFICATION_EPSIZE, 
                    .Type = EP_TYPE_CONTROL, 
                    .Banks = 1, 
                }, 
        }, 
};
