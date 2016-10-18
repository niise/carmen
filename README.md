# carmen

## Configure
<pre>
./configure --noWerror
</pre>
## make it
<pre>
make -j8
</pre>
## Run the simulator (Enter carmen-0.7.4-beta/bin)
<pre>
./central &
./param_daemon -r pioneer-I ../data/freiburg.map &
./simulator &
./robot &
./localize &
./navigator &
./robotgui &
./navigatorgui &
</pre>
