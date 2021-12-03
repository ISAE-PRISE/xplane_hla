# SMARTIES XPLANE HLA PLUGIN

Implements an HLA simulation for an F16 aircraft.
Each federate (HLA simulator) contains a part of the whole simulation (FDM, Cockpit, ...)

### Dependencies

Dependencies:
- C++ compiler is required (only tested on Linux based systems)
- CERTI (or any other HLA compliant RTI) https://savannah.nongnu.org/projects/certi

### Installation

This is a traditional cmake project build with CERTI

- source <CERTI_INSTALL_PATH>/share/scripts/myCERTI_env.sh
- export PLUGIN_PATH=<YOUR_INSTALL_PATH_TO_XPLANE_PLUGINS>
- cd <BUILD_DIRECTORY>
- cmake -DCMAKE_INSTALL_PREFIX=<INSTALL_DIRECTORY> <HLA_XPLANE_PLUGIN_SOURCE_DIRECTORY>
- make install

## Authors

Jean-Baptiste Chaudron (ISAE-SUPAERO)

## Contacts

jean-baptiste.chaudron@isae-supaero.fr

## License

This project is distributed as an open-source software under GPL version 3.0


## Notes

On Fedora 23, 28 and 29:

Error fatal error: gnu/stubs-32.h: No such file or directory on Fedora

Problem that Xplane Plugin is compiled for 32 bit version as well;
Solution, install 32 bits compliant libraries  
- sudo dnf install libstdc++-devel.i686
- sudo dnf install  glibc-devel.i686





