## Project name: sai-simulation (version: 2.0.0)

This project uses Semantic Versioning (http://semver.org/).

### Project short description:

The Sai-Simulation library implements a complete multi-articulated body 
simulation system with the following components:

* Articulated body specification
* Time-stepping integrator
* Multi-point contact detection
* Multi-point multi-body collision resolution
* Multi-point multi-body contact resolution (with Coulomb friction)

### SAI library dependencies:

Chai3d: https://github.com/manips-sai-org/chai3d
sai-urdfreader : https://github.com/manips-sai-org/sai-urdfreader
sai-model : https://github.com/manips-sai-org/sai-model

### Other dependencies

freeglut3, libxmu, libstdc++.

### Installation instructions:

```
mkdir build && cd build
cmake .. && make -j4
```

### Uninstallation instructions: 

```
rm -r build
rm -r ~/.cmake/packages/SAI-SIMULATION
```

### Getting started:

Take a look at sample applications under examples/.
You can run them from build/examples/.

## License

This software is distributed under the terms of the Stanford Academic Software License Agreement For SAI

## Documentation

The documentation can also be accessed online at the following [link](https://manips-sai-org.github.io/sai-simulation/)

It can also be generated locally with doxygen:

```
cd docs
doxygen
```

### Project contributors:

* Kyong-sok Chang
* Diego Ruspini
* Luis Sentis
* Jaehong Park
* Francois Conti
* Shameek Ganguly
* Mikael Jorda
