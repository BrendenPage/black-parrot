#!/usr/bin/env perl


use strict;
use warnings;
use FileHandle;

my $fh=FileHandle->new();
my $outFH=FileHandle->new();

my %hash=();

foreach my $test (qw(stringsearch1 prime sglib-listinsertsort fdct dijkstra aha-compress)) {
# foreach my $test (qw(stringsearch1 prime sglib-listinsertsort aha-compress)) {

    foreach my $depth ("no-prefetch", 2, 4, 8) {
    # foreach my $depth ("no-prefetch") {
	
	print `make -C bp_top/syn clean`;
	die "$!\n" if $?;
	
	open($fh,"<bp_me/src/v/dev/bp_me_cache_slice.sv") || die "Cant read bp_me/src/v/dev/bp_me_cache_slice.sv : $!\n";
	open($outFH,">bp_me/src/v/dev/bp_me_cache_slice.sv.NEW") || die "Cant write read bp_me/src/v/dev/bp_me_cache_slice.sv.NEW : $!\n";
	while(<$fh>) {
	    if ($depth ne "no-prefetch" && /localparam prefetch_buffer_depth_p\s*=\s*/) {
		print $outFH "localparam prefetch_buffer_depth_p = $depth;\n";
		next;
	    }
	    
	    # // assign stride = '0;
	    # // assign stride = {{(daddr_width_p-7){1'b0}},7'b1000000};
	    # assign stride = {{(daddr_width_p-lg_offsets_p-block_offset_width_lp){1'b0}},{offset},{block_offset_width_lp{1'b0}}};

	    if (/assign\s+stride\s*=\s*\'0;/) {
		if ($depth eq "no-prefetch") {print $outFH "assign stride = '0;\n"}
		else                         {print $outFH "// assign stride = '0;\n"}
		next;
	    }
	    if (/assign\s+stride\s*=\s*\{\{\(daddr_width_p-lg_offsets_p-block_offset_width_lp/) {
		if ($depth eq "no-prefetch") {print $outFH "// assign stride = {{(daddr_width_p-lg_offsets_p-block_offset_width_lp){1'b0}},{offset},{block_offset_width_lp{1'b0}}};\n"}
		else                         {print $outFH "assign stride = {{(daddr_width_p-lg_offsets_p-block_offset_width_lp){1'b0}},{offset},{block_offset_width_lp{1'b0}}};\n"}
		next;
	    }
	    
	    
	    print $outFH $_;
	}
	close($fh);	
	close($outFH);
	
	`cp bp_me/src/v/dev/bp_me_cache_slice.sv.NEW bp_me/src/v/dev/bp_me_cache_slice.sv.$depth`;
	die "$!\n" if $?;
	`rm -f bp_me/src/v/dev/bp_me_cache_slice.sv`;
	die "$!\n" if $?;
	`mv bp_me/src/v/dev/bp_me_cache_slice.sv.NEW bp_me/src/v/dev/bp_me_cache_slice.sv`;
	die "$!\n" if $?;
	
	open($fh,"make -C bp_top/syn build.v sim.v SUITE=beebs PROG=$test |") || die "cant exec mtbb PROG=$test : $!\n";
	
#miss_ctr:        524
#cache_ctr:       6133
#NBF loader done!
	
	
	my $last_miss_ctr_1="";
	my $last_cache_ctr_1="";
	my $last_miss_ctr_2="";
	my $last_cache_ctr_2="";
	my $hit_NBF_loader=0;
	my $clk_is_next=0;
	my $hit_core0_fsh=0;
	my $stride_ct=0;
	my $stride_total=0;
	while(<$fh>) {
	    print "($test) ($depth) |$_";
	    if (! $hit_NBF_loader) {
		if (/^miss_ctr:\s+(\S+)/) {$last_miss_ctr_1=$1}
		if (/^cache_ctr:\s+(\S+)/) {$last_cache_ctr_1=$1}
	    }
	    if (! $hit_core0_fsh) {
		if (/^miss_ctr:\s+(\S+)/) {$last_miss_ctr_2=$1}
		if (/^cache_ctr:\s+(\S+)/) {$last_cache_ctr_2=$1}
	    }
	    if (/^NBF loader done\!/) {
		$hit_NBF_loader=1;
	    }
	    if ($hit_NBF_loader) {
		if(/^current stride:\s+(\S+)/) {
		    $stride_ct++;
				$stride_total += $1;
		}
	    }
	    if (/^\[CORE0 FSH\] PASS/) {
		$hit_core0_fsh=1;
	    }
	    if (/^\[CORE0 STATS\]/) {
		$clk_is_next=1;
		next;
	    }
	    if ($clk_is_next) {
		if (/clk\s*:\s*(\d+)/) {$hash{$test}{$depth}{'clk'}=$1}
		if (/instr\s*:\s*(\d+)/) {$hash{$test}{$depth}{'instr'}=$1 ; $clk_is_next=0}
	    }
	    
	    
	    
	}
	my $stride_avg=$stride_total/$stride_ct;
	my $delta_miss=$last_miss_ctr_2-$last_miss_ctr_1;
	my $delta_cache=$last_cache_ctr_2-$last_cache_ctr_1;
	$hash{$test}{$depth}{'delta_miss'}=$delta_miss;
	$hash{$test}{$depth}{'delta_cache'}=$delta_cache;
	$hash{$test}{$depth}{'stride_avg'}=$stride_avg;

#    print "BRENDEN ($depth) -------|last miss_ctr1  = $last_miss_ctr_1\n";
#    print "BRENDEN ($depth) -------|lass cache_ctr1 = $last_cache_ctr_1\n";
#    print "BRENDEN ($depth) -------|last miss_ctr2  = $last_miss_ctr_2\n";
#    print "BRENDEN ($depth) -------|lass cache_ctr2 = $last_cache_ctr_2\n";
	print "BRENDEN ($depth) -------|DELTA miss_ctr  = $delta_miss\n";
	print "BRENDEN ($depth) -------|DELTA cache_ctr = $delta_cache\n";
    }
}
foreach my $test (qw(stringsearch1 prime sglib-listinsertsort matmult-float matmult-int fdct dijkstra aha-compress)) {
	# print "($test) instr           = " . $hash{$test}{'no_prefetch'}{'instr'} . "\n";
	# print "($test) DELTA cache_ctr = " . $hash{$test}{'no_prefetch'}{'delta_cache'} . "\n";
    foreach my $depth (sort {$a<=>$b} keys %{ $hash{$test} }) {
	print "($test) ($depth) clk             = " . $hash{$test}{$depth}{'clk'} . "\n";
	print "($test) ($depth) DELTA miss_ctr  = " . $hash{$test}{$depth}{'delta_miss'} . "\n";
	print "($test) ($depth) average stride  = " . $hash{$test}{$depth}{'stride_avg'} . "\n";
	print "----------------------------------------------------\n";
    }
}

