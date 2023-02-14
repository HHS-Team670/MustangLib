package frc.team670.mustanglib.utils;

import frc.team670.robot.constants.CodeVersion;

public final class CodeVersionStringGenerator {
    public static String genVersionString() {
        String str = "";

        if (CodeVersion.DIRTY == 1) {
            str = CodeVersion.GIT_BRANCH + " (dirty) at " + CodeVersion.BUILD_DATE;
        } else {
            str = CodeVersion.GIT_BRANCH + " at " + CodeVersion.BUILD_DATE + " - commit " + CodeVersion.GIT_SHA.substring(0, 8);
        }

        return str;
    }
}
