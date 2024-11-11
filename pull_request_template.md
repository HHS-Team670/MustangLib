# Description

Removed the path finder folder and related files from mustanglib.
The files were not being used anywhere except fieldConstants (outdated file)
Change does not affect any subsystems. 

Also fixed errors from 2025 main branch
- Class Name does not match file name
- SetDesiredHeading readded to XboxSwerveDrive

## Affected Subsystems

Please check the subsystems that will be affected by this change

- [ ] DriveBase
- [ ] Shooter
- [ ] Intake
- [ ] Climber
- [ ] Indexer
- [ ] VendorDeps Update
- [X] Library Update
- [ ] Vision
- [ ] Auton
      
# Steps to Test

Please describe the steps you took to verify this feature/change/bux fix.

# Checklist:

- [x] My changes pass ./gradlew build
- [x] My code follows the style guidelines of this project
- [x] I have performed a self-review of my code
- [x] I have commented my code, particularly in hard-to-understand areas
- [x] I have made corresponding changes to the documentation
