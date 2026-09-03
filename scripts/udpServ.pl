#!/usr/bin/perl -w
use strict;
use IO::Socket;
use Socket qw(:all);

my ($sock, $oldmsg, $newmsg, $hisaddr, $hishost, $MAXLEN, $PORTNO);
$MAXLEN = 1024;
$PORTNO = 54321;

# Create the UDP socket
$sock = IO::Socket::INET->new(
    LocalPort => $PORTNO,
    Proto     => 'udp',
    Type      => SOCK_DGRAM
) or die "socket: $@";

print "Awaiting UDP messages on port $PORTNO\n";

$oldmsg = "Hello Client";

# Main server loop
while ($sock->recv($newmsg, $MAXLEN)) {
    # Get the client's address information
    my ($port, $ipaddr) = sockaddr_in($sock->peername);
    $hishost = gethostbyaddr($ipaddr, AF_INET);
    if ( !$hishost )  {
        $hishost = "unknown";
    }

    print "Client $hishost:$port said '$newmsg'\n";
    if ( $newmsg )  {
        if ( $newmsg =~ "dx" )  {
            my $coef = substr $newmsg, 2;
            $newmsg = "dx $coef";
        }
        elsif ( $newmsg =~ "dy" )  {
            my $coef = substr $newmsg, 2;
            $newmsg = "dy $coef";
        }

        system("/work/scripts/rgbir_ctl.sh $newmsg");
    }

    # Send a response back to the client
    $sock->send($oldmsg);
    $oldmsg = "[$hishost] $newmsg";
}

die "recv: $!";

