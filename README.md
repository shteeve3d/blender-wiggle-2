<!--
Keep this document short & concise,
linking to external resources instead of including content in-line.
See 'release/text/readme.html' for the end user read-me.
-->

# Wiggle 2

Wiggle 2 is a ground up rewrite of the [wiggle bones add-on](https://github.com/shteeve3d/blender-wiggle) for Blender.

## Features

### New physics logic.
- Wiggling now behaves more realistically, especially when simulating simple ropes or chains.

### Pinning.
- Using a damped track constraint on a wiggling bone pins it to its target, with other bones responding accordingly.
!["Pinning"](/images/pinning.png?raw=true "Pinning")

### Collision support.
- Bones can collide with a specified mesh or collection, and respond with friction, bouncing, or even stickiness.
!["Collision"](/images/collision.png?raw=true "Collision")

### Linking and library overrides.
- Wiggle 2 works properly on library linked assets, with overrides allowing you to tune your wiggle to each scene.

### Baking refinements.
- One click bake converts an objects visible wiggle bones into key frames. Preroll options allow your simulation to settle. Or use it with the timeline looping option for generating seamless wiggle on animated loops. 

### Refreshed interface.
- Everything can be managed from a single panel in the 3d animation view for streamlined, fullscreen workflows.

## Usage
WIP.

License
-------

Wiggle 2 as a whole is licensed under the GNU General Public License, Version 3.
Individual files may have a different, but compatible license.
