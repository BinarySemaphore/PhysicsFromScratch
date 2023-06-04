# PhysicsFromScratch

## Table of Contents
1. [Info](#info)
   1. [Released Builds](#released-builds)
   1. [Preface](#preface)
   1. [Intent](#intent)
1. [Setup and Usage](#setup-and-usage)
1. [Examples](#examples)
1. [Documentation](#documentation)
   1. [Source Web Documentation](#source-web-documentation-via-github-pages)
1. [Updating Web Documentation for GitHub Pages](#updating-web-documentation-for-github-pages)
   1. [Generating HTML Web Docs](#generating-html-web-docs)
   1. [Update GitHub Pages](#update-github-pages)


## Info
* **Author(s)**: BinarySemaphore
* **Version**: 0.1.1
* **Description**: Physic simulation from scratch (in Unity/Ylands).

### Released Builds
* Not yet released

### Preface
I started making this in a game called "[Ylands](https://ylands.com/)".
This was in their in-game editor to allow blocks to follow a physics simulation.
The end goal, to destroy "blueprints" (structures built in the game - think Minecraft buildings) by throwing other blocks or an explosive force.

I was posting progress on the Ylands official discord:
* [Original post](https://discord.com/channels/243416130759163904/1068142260501348384/1107579623530844210)
* [Adding octree](https://discord.com/channels/243416130759163904/329978085547966464/1108021164984520757)
* [Update on octree](https://discord.com/channels/243416130759163904/329978085547966464/1108131113181913089)
* [Dynamic collisions](https://discord.com/channels/243416130759163904/1068142260501348384/1110135540277321728)
* [Fixed collisions](https://discord.com/channels/243416130759163904/1068142260501348384/1110178328477970482)

I decided to continue my work in Unity to allow for additional debugging and to expand on it later.

See the `from-ylands` branch for the ported code.

### Intent
Continue working towards a simple physics simulation in Unity with good performance.

Eventually port back to Ylands and continue there until a custom tool can be released.

After release, continue in Unity and add more complexity until I'm tired of working on physics stuff.


## Setup and Usage
Describe how everything fits together at some point.


## Examples
Walk through some examples one day.


## Documentation

### Source Web Documentation (via GitHub Pages)
[Web Docs](https://binarysemaphore.github.io/PhysicsFromScratch/)


## Updating Web Documentation for GitHub Pages

### Generating HTML Web Docs
Open Doxygen's [Doxywizard](https://www.doxygen.nl/index.html)
* File > Open and open `Doxyfile` located in ["./Docs/"](../main/Docs/)
* Click the "Run" tab
* Select "Run doxygen"
* Everything in ["./Docs/html"](../main/Docs/html) should now be updated

Commit the updated docs, push to your branch, and create a pull request

### Update GitHub Pages
Note: *It can take 10 minutes for GitHub Pages to update.*

#### First Time Setup
Create orphan branch for GitHub Pages:
* `git checkout --orphan gh-pages`
 
#### Update `gh-pages` Branch
Update GitHub Pages from `main` branch:
* Checkout main, fetch and pull latest
* Update `gh-pages` branch:
  * `git merge main`
   
Clean up:
* Remove everything except "./Docs/" directory
  * Example: `rm -rf Assembly-CSharp.csproj Assets/ Library/ Logs/ Packages/ ProjectSettings/ PhysicsFromScratch.sln README.md UserSettings/ obj/`
* Move everything from "./Docs/html/" into root directory:
  * `mv Docs/html/* ./`
  * `rm -rf Docs/`
   
Send it:
* Commit and push `gh-pages` branch:
  * `git add ./*`
  * `git commit -a -m 'Updated Docs for GitHub Pages'`
  * `git push origin gh-pages`
