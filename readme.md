# Box2DWeb

## Box2DWeb Redux

A redux of mikolalysenko's npm listing of Uli Hecht's port of Box2DFlash which is the flash port of Erin Catto's box2d library.

Try to say that in one breath!

Box2DWeb 2.1alpha has a ton of code wrapped up in some dated pre-html5 bindings.
[Mikolalysenko] (https://www.npmjs.com/package/box2dweb) fixed the Object.defineProperty issue,
and exposed the postDefs object. I based an automated process on this version.

Phase I: automated recompilation of the code to use modern idioms

    POC, Complete.
    The redux process iterates over the live Box2D object, extracting the source code for all methods.
    This code is used to generate new class definitions, 1 class per file. Multiple initializations are
    reduced to 1 constructor function.This cuts the function call overhead in half when allocating new
    framework objects.


Phase II: performance cleanup

    In Progress.
    Set Strict Mode.
    Fix:
        ==/!= for non null values
        undefined vars
        define function inside for/next
        new Array(1,...)
    Performance:
        tune array initializations
        remove dead code
        remove unnecessary parseInt
        add properties to prototypes - this should also reduce overhead:
        1. Avoids unnecessary initialization code
        2. Results in fewer hidden classes


## Performace

Performance is a slippery slope. I've created a simple test, taken from Asteroids,
that creates 10,000 bullets. The classes involved have completed phase II.

Typical results in chrome:

    Box2D - (5269, 5113, 5167, 5488, 5336) = 26373 ms
    Redux - (3541, 3575, 3550, 3421, 3622) = 17709 ms 3063, 2794, 3137, 3300, 3345 = 15639 ms

2718


## Install

```bash
$ git clone https://github.com/darkoverlordofdata/Box2dWeb.git
$ cd Box2dWeb
$ npm install
$ cake redux
```
