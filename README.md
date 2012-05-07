This is a reasonable stable kernel for [Nokia N810](http://fi.wikipedia.org/wiki/Nokia_N810),
based on mainline 2.6.38.8 version with the patches from
[openwrt](https://github.com/mirrors/openwrt). Looks like
openwrt folks are the most active [users of recent mainline
kernels on Nokia N810](http://bues.ch/cms/hacking/n810-openwrt.html)
and maintain their patchset in a more or less good shape.

<b>!!! WARNING !!!</b> be careful with battery charging and use it at your own risk. It may
be safer to first fully charge the battery with maemo firmware and then experiment with
running gentoo only for a few hours before it runs out of juice. To be honest, I have not
experimented with this stuff much (or more like have not experimented with it at all).
Though you may search for openwrt n810 battery charging information, read the guides
and try to get this stuff working properly.

The included [n810_nfs_defconfig](https://github.com/ssvb/linux/blob/n810-2.6.38/arch/arm/configs/n810_nfs_defconfig)
is preconfigured to boot the system over NFS from <b>/mnt/armv6-nfs-root</b> directory on your desktop PC (see
CONFIG_CMDLINE option if you want to tweak this).

## Using gentoo linux with this kernel and booting it over NFS

### Step one: compile the kernel and flash it to N810

First of all, you need an ARM crosscompiler. If you don't have it already, then you can do
```bash
# emerge dev-util/crossdev
# crossdev -S arm-none-linux-gnueabi
```

Then we can compile the kernel itself. It is also easy:
```bash
$ make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- n810_nfs_defconfig
$ make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- -j8 bzImage
```

After this step, you get a kernel binary <b>arch/arm/boot/zImage</b>. Using the
[maemo flasher tool](http://tablets-dev.nokia.com/maemo-dev-env-downloads.php), this
kernel can be either just booted:
```bash
$ flasher-3.5 -k arch/arm/boot/zImage -l -b
```
or flashed to the device, replacing the old kernel:
```bash
$ flasher-3.5 -k arch/arm/boot/zImage -f
```
In both cases, the flasher will wait for the N810 to be turned on or rebooted while
it is plugged to your PC via usb cable. Alternatively it should be possible
to use the [open source 0xFFFF flasher](http://nopcode.org/0xFFFF/).

Now the kernel is ready to boot the system over NFS from your PC.

### Step two: setting up the network on your desktop PC

First we want to set a network connection between N810 and your PC. Let's configure <b>usb-n810</b> network
interface and set it to use <b>192.168.4.15</b> IP address for N810 and <b>192.168.4.14</b> IP address for your PC.

First edit <b>/etc/udev/rules.d/70-persistent-net.rules</b> and add the following line:
```bash
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="9a:49:12:0f:53:14", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="usb*", NAME="usb-n810"
```
This ensures that the device with MAC address <b>9a:49:12:0f:53:14</b> (see CONFIG_CMDLINE in n810_nfs_defconfig)
will be assigned a unique name <b>usb-n810</b>.

Then edit <b>/etc/conf.d/net</b> and add the following line to it:
```text
config_usb_n810=( "192.168.4.14 netmask 255.255.255.0" )
```

Make a symlink:
```bash
# ln -s /etc/init.d/net.lo /etc/init.d/net.usb-n810
```

Edit <b>/etc/hosts</b> and add
```text
192.168.4.15	n810
```

Run iptables to configure NAT and give N810 access to the Internet via your PC,
more details are in [Gentoo Home Router Guide](http://www.gentoo.org/doc/en/home-router-howto.xml)

```bash
# First we flush our current rules
iptables -F
iptables -t nat -F

# Setup default policies to handle unmatched traffic
iptables -P INPUT ACCEPT
iptables -P OUTPUT ACCEPT
iptables -P FORWARD DROP

# Copy and paste these examples ...
export LAN=usb-n810
export WAN=eth0

# Finally we add the rules for NAT
iptables -I FORWARD -i ${LAN} -d 192.168.0.0/255.255.0.0 -j DROP
iptables -A FORWARD -i ${LAN} -s 192.168.0.0/255.255.0.0 -j ACCEPT
iptables -A FORWARD -i ${WAN} -d 192.168.0.0/255.255.0.0 -j ACCEPT
iptables -t nat -A POSTROUTING -o ${WAN} -j MASQUERADE

# Tell the kernel that ip forwarding is OK
echo 1 > /proc/sys/net/ipv4/ip_forward
for f in /proc/sys/net/ipv4/conf/*/rp_filter ; do echo 1 > $f ; done
```

### Step three: preparing gentoo rootfs and booting the system

Now we need to get armv6 gentoo [stage3 tarball from one of the mirrors](http://trumpetti.atm.tut.fi/gentoo/releases/arm/autobuilds/current-stage3-armv6j/)
and unpack it to <b>/mnt/armv6-nfs-root</b>, also unpack [portage snapshot](http://trumpetti.atm.tut.fi/gentoo/releases/snapshots/current/)
and install nfs-utils:
```bash
# mkdir /mnt/armv6-nfs-root
# tar -xjpf stage3-armv6j-20120316.tar.bz2 -C /mnt/armv6-nfs-root
# tar -xjf portage-latest.tar.bz2 -C /mnt/armv6-nfs-root/usr
# emerge net-fs/nfs-utils
# rc-update add nfs default
```

Then edit <b>/etc/exports</b> adding
```text
/mnt/armv6-nfs-root 192.168.4.0/24(rw,no_root_squash,no_all_squash,no_subtree_check,async)
```
And start nfs:
```bash
# /etc/init.d/nfs start
```

Now the system should be bootable on N810, but we also want ssh access to it. In order to do this,
we can configure sshd to
start on system boot and add your public ssh key:
```bash
# ln -s /etc/init.d/sshd /mnt/armv6-nfs-root/etc/runlevels/default
# mkdir /mnt/armv6-nfs-root/root/.ssh
# cp /home/yourusername/.ssh/id_rsa.pub /mnt/armv6-nfs-root/root/.ssh/authorized_keys
```

Finally in order to get some kind of DNS, <b>/etc/resolv.conf</b> is better to be also updated:
```bash
echo "nameserver 8.8.8.8" > /mnt/armv6-nfs-root/etc/resolv.conf
```

That's all. You can connect N810 via usb cable, turn it on and wait till gentoo boots. Then do "ssh root@n810".
