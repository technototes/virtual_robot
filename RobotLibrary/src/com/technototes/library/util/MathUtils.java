package com.technototes.library.util;

public class MathUtils {

    public static double getMax(double... args) {
        double max = args[0];
        for (int i = 1; i < args.length; i++) {
            max = Math.max(max, args[i]);
        }
        return max;
    }

    public static int getMax(int... args) {
        int max = 0;
        for (int i = 1; i < args.length; i++) {
            max = Math.max(args[i - 1], args[i]);
        }
        return max;
    }

    public static double safeATan(double a, double b) {
        return (b == 0) ? ((a >= 0) ? Math.PI : 0) : Math.atan(a / b);
    }

    public static double pythag(double a, double b) {
        return Math.sqrt(a * a + b * b);
    }
}
