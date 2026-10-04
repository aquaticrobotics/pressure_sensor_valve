Import("env")

if env.subst("$PIOENV") == "esp32ota":
    import os

    env.Replace(
        UPLOADER=os.path.join(env.subst("$PROJECT_DIR"), "tools", "espota_fixed.py")
    )
    env.Append(UPLOADERFLAGS=["--host_port", "3232"])
