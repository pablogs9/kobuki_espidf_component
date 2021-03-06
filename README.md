# Kobuki component for ESP-IDF

This component has been tested in ESP-IDF v4.1 and v4.3

## Example

In order to test a int32_publisher example:

```bash
. $IDF_PATH/export.sh
cd examples/basic_movement
idf.py menuconfig
# Set your Kobuki configuration
idf.py build 
idf.py flash 
idf.py monitor 
```


## Purpose of the Project

This software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards, e.g., ISO 26262.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open-source components included in ROS 2 system_modes,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Known Issues/Limitations

There are no known limitations.
