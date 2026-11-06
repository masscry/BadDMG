# SDL's Java glue is looked up from native code by class and method name
# (JNI_OnLoad/RegisterNatives and GetStaticMethodID), so it must not be
# renamed or removed.
-keep class org.libsdl.app.** { *; }
