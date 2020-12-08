# Intel realsense camera input for macos OBS 

## Dependencies

`brew install librealsense`

## Building

For now checkout this repo into `<obs-studio-home>/plugins/mac-input-realsense`.

## Bundling

my crude changes to `CI/full-build-macos.sh` are in `ci.diff`

## TODO
- publish data from other sensors and thus enable filters using them (i don't think input source should be doing stuff like virtual green screen etc)