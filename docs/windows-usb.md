# Windows sensor enumeration

The CV1 camera has two USB interfaces, but that does not imply two Windows device nodes. A read-only inspection of a USB 3 CV1 sensor's descriptors found device class/subclass/protocol `EF/02/01` and this interface association descriptor (IAD):

```text
08 0B 00 02 FF 03 00 04
      |  |  |  |  |
      |  |  function class/subclass/protocol: FF/03/00
      |  interface count: 2
      first interface: 0
```

The individual control and video interfaces use `FF/01/00` and `FF/02/00`. Microsoft's [interface collection documentation](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/support-for-interface-collections) specifies that Usbccgp groups the IAD into one function node and derives its compatible IDs from the IAD. Thus `MI_00` with compatible ID `USB\Class_FF&SubClass_03&Prot_00` represents both interfaces; WinUSB exposes the video interface as an associated interface. Waiting for an independent `MI_01` node would never finish.

The readiness check accepts that specific CV1 function identity, not any lone interface. Without evidence of grouping it still requires the separate interface nodes. It also retains the started-state, problem-code, arrival-identity and active device-interface checks. Synthetic regressions cover grouped, separate, incomplete and failed layouts.

These descriptors and documented Windows behavior explain the software fix; they do not establish that a physical Windows host can stream this camera. Actual WinUSB transfers, USB 2 layouts, hub bandwidth, replugging and sleep/resume still need hardware testing. The descriptor bytes above contain no serial number, room calibration or personal device path.
