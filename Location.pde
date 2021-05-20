import java.util.*;
import java.io.*;

public class Location{
    public float x, y;
    public Location(float x, float y){
        this.x=x;
        this.y=y;
    }
    public boolean equals(Location candidate){
        return candidate.x==x && candidate.y==y;
    }
    public float distance(Location one, Location two){
        return (float)(Math.ceil(Math.sqrt(Math.pow( (one.x-two.x),2) + Math.pow( (one.y-two.y),2))));
    }
    public Location add(Location addend){
        return new Location(this.x+addend.x,this.y+addend.y);
    }
    public Location subtract(Location subtractend){
        return new Location(this.y-subtractend.y, this.x - subtractend.x);
    }
    public float magnitude(){
        return (float)(Math.sqrt(this.x*this.x + this.y*this.y));
    }
    public boolean is_greater_or_equal(Location one, Location two){
        return ( one.x >= two.x && one.y >= two.y);
    }
}
