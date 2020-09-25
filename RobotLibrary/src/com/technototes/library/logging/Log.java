package com.technototes.library.logging;

import java.lang.annotation.*;

import static java.lang.annotation.ElementType.*;

@Repeatable(Log.Logs.class)
@Retention(RetentionPolicy.RUNTIME)
@Target(value={FIELD, LOCAL_VARIABLE, METHOD})
public @interface Log {
    int entry() default -1;

    String name() default "Entry";

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface Logs {
        Log[] value();
    }
}
