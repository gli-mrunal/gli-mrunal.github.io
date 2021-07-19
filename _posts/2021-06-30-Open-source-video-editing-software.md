---
title: Installing Pitivi - Open Source Video Editing Software 
author: Cotes Chung
date: 2019-08-08 14:10:00 +0800
categories: [Blogging, Ubuntu]
tags: [Ubuntu]
---
[Pitivi](https://www.pitivi.org/) is an open Source Video Editing Software for Linux.
Currently not available for Windows.

## Step 1: Setup in Ubuntu before installing Pitivi

Go to this [flatpak](https://flatpak.org/setup/)link to select the `Ubuntu` icon for [flatpak setup for Ubuntu](https://flatpak.org/setup/Ubuntu/) instructions.


# Ubuntu Quick Setup

- Follow these simple steps to start using Flatpak
    To install Flatpak on Ubuntu 18.10 (Cosmic Cuttlefish) or later, simply run:

    ```sh
        sudo apt install flatpak
    ```

- Add the Flathub repository:

    ```sh
    flatpak remote-add --if-not-exists flathub https://flathub.org/repo/flathub.flatpakrepo

    ```
- Restart

    To complete setup, restart your virtual machine system. 

## Step 2: Install Pitivi

Now all you have to do is install some apps! 

[Installation instructions](https://flathub.org/apps/details/org.pitivi.Pitivi) link.

Command line instructions
- Install:

    Make sure to follow the setup guide before installing
    ```sh
    flatpak install flathub org.pitivi.Pitivi

    ```
- Run:

```sh
    flatpak run org.pitivi.Pitivi
```