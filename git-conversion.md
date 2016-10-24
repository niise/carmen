CARMEN git repository
=====================

This is a git repository for CAEMEN - the Carnegie Mellon Robot Navigation Toolkit.
For more information, please refer to their [website](http://carmen.sourceforge.net/).

git conversion
--------------

The original source code for the project is hosted on Sourceforge, in a Subversion
repository. This git repository is done for Acer's research purpose, with the
following steps:

1. Using `git svn` to clone project souce.
```
git svn clone svn://svn.code.sf.net/p/carmen/code carmen
cd carmen
```

2. The newly cloned git repo contains a lot of empty commits, remove them:
```
git rebase --root
```

