#ifndef MEASUREMENT_H
#define MEASUREMENT_H

struct Measurement {
  float gate;
  float source;
  float drainADC;
  float drain;
  float vgs;
  float vds;

  float baseADC;
  float emitterADC;
  float base;
  float emitter;
  float vbe;

  float opampADC;
  float opamp;
  float baseResDrop;
  float ib_mA;
  float vdsTheory;
};

#endif