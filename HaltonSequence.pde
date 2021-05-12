import java.io.*;
import java.util.*;

float X;
float Y;
boolean go = true;

void setup() {
  size(400,400);
}

public float generateHalton(int i, int b) {
  float r = 0;
  float f = 1;
  while(i > 0) {
    f = f/b;
    r = r + f*(i % b);
    i = i/b;
  }
  return r;
}
void draw() {
  if(go) {
    background(255,255,255);
  
    fill(255,0,0);
    for(int i = 0; i < 255; i ++) {
      X = generateHalton(i,2);
      Y = generateHalton(i,3);
      System.out.println(X + ", " + Y);
       ellipse(X*400,Y*400,5,5);
    }
  go = false;
  }
}
