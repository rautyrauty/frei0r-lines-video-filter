# frei0r straight lines video filter
#### This is a video filter using the frei0r cross-platform framework. He turns frames into linear art.
## About frei0r
- [wiki](https://en.wikipedia.org/wiki/Frei0r)
- [github](https://github.com/dyne/frei0r)
- [site](https://frei0r.dyne.org/)
## Usage with ffmpeg
```
ffmpeg -loglevel debug -i input_file -vf "frei0r=straightlines:number_of_lines|sample_size|extreme_line_mode|long_line_mode" -t output_file_seconds output_file
```
## Usage with ffmpeg example
```
ffmpeg -loglevel debug -i videoplayback.mp4 -vf "frei0r=straightlines:2000|10|n|y" -t 30 lovebff.mp4
```
## Parameters

- Number of Lines - Default: 1000
- Sample size - Number of attempts to build a line. Default: 10
- Extreme Line Mode - Should draw the extreme lines? Default: false
- Long Line Mode - Should draw lines to the end of the screen? Default: true

## Example
https://youtu.be/T90l252W8g0

## Credits
I am grateful to 
- [BadAppleFLSL](https://github.com/CameronChoy/BadAppleFLSL) for the algorithm, the idea and the open source code
- [frei0r](https://frei0r.dyne.org/) (GPL-2.0 license) for the open source code and a wonderful cross-platform framework for creating video filters in C++
- [ffmpeg](https://www.ffmpeg.org/) for the open source code