class ConvexPolygon{
  ArrayList<PVector> vrt;
  public ConvexPolygon(ArrayList<PVector> vrt){
    this.vrt = vrt;
  }
  public boolean isContained(PVector one){
    float angle = 0;
    for(PVector nodes : vrt){
      angle += atan( (nodes.y-one.y)/(nodes.x-one.x) );
    }
    return (angle == 2*PI);
  }
}
