class GType{
  PVector evec;
  float COST_TO_REACH;
  float HEURISTIC_COST;
  public GType(PVector evec, float COST_TO_REACH){
    this.evec = evec;
    this.COST_TO_REACH = COST_TO_REACH;
  }
  public GType(PVector evec, float COST_TO_REACH, float HEURISTIC_DIST){
    this.evec = evec;
    this.COST_TO_REACH = COST_TO_REACH;
    this.HEURISTIC_COST = HEURISTIC_COST;
  }
}
