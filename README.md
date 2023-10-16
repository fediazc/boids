# Boids Demo

A simple demonstration of the 'boids' algorithm, made with Webassembly (C + emscripten).

# Running the project

Make sure you have emscripten installed and set up.

Run the makefile in `src/`. The build files will be located in the `build/` directory.

To spin up a web server with the demo, run

```
emrun ./build/demoapp.html
```

Then visit http://localhost:6931/demoapp.html on a browser.
