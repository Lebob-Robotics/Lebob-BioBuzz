# Solo auto viewer

Every branch of the solo auto (`auto/SoloAuto`, Solo Red / Solo Blue) on one timeline, on AdvantageScope's 2026-2027
field picture: one robot until the Hive check, then "Not tipped" and "Tipped", and from the recheck at our wall's
Flower "Far still up" (tipped, but the far Cell has not tipped back, so it is shot again from the park). The drives
are `SoloAuto`'s own Pedro paths on `SoloAutoSimTest`'s simulated robot, whose drive model matches the regionals
autos' drives to 0.05 s on average.

    WRITE_SOLO_TRACE=1 ./gradlew :TeamCode:testDebugUnitTest --offline --tests '*SoloAutoSimTest'
    node tools/solo/build.js          # solo.html: open it in a browser, play or drag the slider
    node tools/solo/build.js --gif    # and solo.gif (needs ImageMagick's magick)
    node tools/solo/build.js --mp4    # and solo.mp4, real time (magick and ffmpeg: $FFMPEG or the PATH; ~2 min)

The first writes `solo-trace.js`. `build.js` takes the field from AdvantageScope's assets folder, which AdvantageScope
fills once it has run online (`SOLO_FIELD_DIR` picks another folder holding `image.png` and `config.json`). The
outputs are generated, so git ignores them.

- `view.js`: draws one frame as SVG, for the page, the GIF and the MP4.
- `build.js`: finds the field, writes the page, the GIF and the MP4.
