package com.technototes.library.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public class Logger {
    public Set<Supplier> entries;
    public Telemetry telemetry;
    public Object root;

    public Logger(Telemetry tel, Object r) {
        root = r;
        telemetry = tel;
        entries = new HashSet<>();
        configure(r);
    }

    public void update() {
        entries.forEach((s) -> {
            telemetry.addLine("" + s.get());
        });
    }

    public void configure(Object root) {
        for (Field field : root.getClass().getDeclaredFields()) {
            try {
                Object o = field.get(root);
                if (o instanceof Loggable) {
                    configure(o);
                } else if (field.isAnnotationPresent(Log.class)) {
                    for (Method m : o.getClass().getDeclaredMethods()) {
                        if (m.isAnnotationPresent(Log.class)) {
                            set(field.getDeclaredAnnotation(Log.class).name(), m, o);
                        }
                    }
                }
            } catch (IllegalAccessException e) {
                continue;
            }
        }
        for (Method m : root.getClass().getDeclaredMethods()) {
            if (m.isAnnotationPresent(Log.class)) {
                set(m.getDeclaredAnnotation(Log.class).name(), m, root);
            }
        }

    }

    public void set(String name, Method m, Object root) {
        entries.add(() -> {
            try {
                return name + ": " + m.invoke(root);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } catch (InvocationTargetException e) {
                e.printStackTrace();
            }
            return null;
        });
    }

}
