package com.technototes.library.logging.entry;

import java.awt.*;
import java.util.function.Supplier;

public class NumberBarEntry extends NumberSliderEntry {
    public NumberBarEntry(String n, Supplier<Number> s, Dimension d, Number mi, Number ma, Number sc) {
        super(n, s, d, mi, ma, sc);
    }
    public NumberBarEntry(String n, Supplier<Number> s, int x, int y, Number mi, Number ma, Number sc) {
        super(n, s, x, y, mi, ma, sc);
    }

    @Override
    public String toString() {
        String r = name+": [";
        for(double i = (double)min; i <= (double)max; i+=(double)scale) {
            if(Math.abs((double)get()-i)*2 < (double)scale){
                r+="#";
            }else if((double)get() < i){
                r+="~";
            }else{
                r+="=";
            }
        }
        return r+"]";
    }

}
