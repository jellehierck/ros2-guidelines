# ROS 2 Guidelines

[![Page Deployment](https://github.com/jellehierck/ros2-guidelines/actions/workflows/ci.yaml/badge.svg)](<https://github.com/jellehierck/ros2-guidelines/actions/>
workflows/ci.yaml)

Deployed website: <https://jellehierck.github.io/ros2-guidelines/>

This repository contains a website with information on ROS 2 and additional guidelines, originally created for the [Nakama Robotics Lab](https://www.utwente.nl/en/et/be/research/nakama_robotics_lab/) at the University of Twente.

The website contents are created in [Obsidian](https://obsidian.md/), and transformed to GitHub Pages using [Quartz](https://quartz.jzhao.xyz/) and the [obsidian-quartz-template](https://github.com/DefenderOfBasic/obsidian-quartz-template).

## Edit page contents with Obsidian

To change page contents, you need to open the page contents in Obsidian. 
These steps assume you can push to the repository (or open an issue/PR!)

Some default plugins are shipped with the page contents to spell check and lint the markdown files. 
They should be enabled by default when you open the vault in Obsidian.

1. Clone this repository to your own PC. Ensure you are on the `main` branch.
2. Open the folder `source/contents/` as Obsidian vault using [the official instructions](https://help.obsidian.md/manage-vaults#Create+vault+from+an+existing+folder).
3. Make your edits.
   - If you add a file, run the Linter plugin in Obsidian to populate it with required tags.
   - If you add a folder, take a look at [Creating a folder](#creating-a-folder) for additional required steps.
4. Update the [Tags](#tags).
5. Commit and push the changes.
6. Wait until the deployment CI action is finished (typically takes <1 minute).
7. See your changes in the deployed website.

### Tags

The Obsidian Linter plugin is set up to automatically include YAML tags to the page to control metadata. The following tags are added:

```yaml
---
publish:                # Set to false to not publish the page yet
title:                  # Alternative title for the page. If not set, the note title is used.
description:            # Short description of the page contents.
permalink:              # Create a permalink for this page relative to the page base, e.g. /fixed/path
aliases:                # Create tags for the page
tags:                   # Create tags for the page
created:                # When the page was created (updated automatically)
modified:               # When the page was last modified (updated automatically)
---
```

### Creating a folder

If you create a folder, it will be shown as a grouped topic inside the Explorer sidebar.
However, you need an additional note with folder metadata for this to work properly.

When you create a folder inside Obsidian, also add an `index.md` note to that folder and run the Linter to populate it with tags.
Change the `title` to the name that you want displayed at the top of the Index page, e.g. the created folder's name.

**TODO: I want to be able to specify the page order using this index note in the future, but that is not possible yet.**

## Edit the website

For any adjustments except [page contents](#edit-page-contents-with-obsidian), you need to edit files in the repository with another editor than Obsidian. 
You could e.g. want to add [Raw HTML pages](https://github.com/DefenderOfBasic/obsidian-quartz-template?tab=readme-ov-file#raw-html-pages) or [Further customize Quartz](https://github.com/DefenderOfBasic/obsidian-quartz-template?tab=readme-ov-file#further-customization).

1. Clone this repository to your own PC. 
   Ensure you are on the `main` branch. 
   These steps assume you can push to the repository (or open an issue/PR!)
2. Open the repository root in your favourite editor, such as Visual Studio Code.
3. Make your edits.
4. Commit and push the changes.
5. Wait until the deployment CI action is finished (typically takes <1 minute).
6. See your changes in the deployed website.

## Authors

The guidelines contents were created by:

- Jelle Hierck (<j.j.hierck@student.utwente.nl>)

## Acknowledgment

This repository is forked from <https://github.com/DefenderOfBasic/obsidian-quartz-template> and was created by [DefenderOfBasic](https://github.com/DefenderOfBasic). See the original repository for more information!
