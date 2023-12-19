*********************************************
© 2023 Nokia                                  
Licensed under the BSD 3-Clause Clear License 
                                              
SPDX-License-Identifier: BSD-3-Clause-Clear   
*********************************************

The cm23 code computes the periodic service sequence (PSS) for the FLAT and LD ingress shapers
described in the manuscript "Lightweight Determinism in Large-Scale Networks", by Andrea Francini,
Raymond Miller, Bruce Cilli, Charles Payette, and Catello (Lelio) Di Martino.

The code builds with the following command:
$ make

The code runs with the following command:
$ ./cm23 < filename.inp

The repository includes multiple examples of .inp files, configured for the FLAT and LD shapers.
This description refers to the files with total reserved rate of 7.2 Gb/s (72% load).

The rows in the .inp file (e.g., des-01-LD-720.inp, which is used for the values shown in the
row examples that follow) have the following meanings (the meanings are fixed and implied by the 
position in the file; changing the order of the rows will cause misconfiguration of the run and 
most likely an early crash):

R1. des-01-720-LD /* Output File Radix                            */
Filename root for the output files generated by a cm23 run (the .pss output file contains
the computed periodic service sequence; the .srv output file counts the number of PSS timeslots
assigned to each queue in the PSS).

R2. 2               /* L1 Scheduler (WF2Q)                          */
Type of scheduler used in the lower layer of the hierarchy (1 for WFQ, 2 for WF2Q; for the
L1 scheduler, WF2Q (2) is to be used for LD, WFQ (1) is to be used for FLAT).

R3. 2               /* L2 Scheduler (WF2Q)                          */
Type of scheduler used in the upper layer of the hierarchy (1 for WFQ, 2 for WF2Q: WF2Q is
to be used for both FLAT and LD).

R4. 10001           /* Number of Slots                              */
Number of timeslots in the periodic service sequence (PSS), increased by 1 unit
(i.e., 10001 means that the PSS contains 10000 slots).

R5. 481             /* Number of L1 scheduling nodes: Queues        */
Number of scheduling nodes receiving service in the bottom layer of the hierarchy
(must be equal to the number of flows + 1 unit, for the virtual empty queue).

R6. 17              /* Number of L2 scheduling nodes: Paths         */
Number of scheduling nodes receiving service in the top layer of the hierarchy
In the case of the LD shaper, it must be equal to the number of network paths used by
actual flows, + 1 unit for the path of the virtual empty queue (17 in the .inp file included here
for the LD case).
In the case of the FLAT shaper, the top node of the hierarchy serves one node per queue, and then
each node in the bottom layer of the hierarchy serves a single queue, so that the hierarchy
reduces in practice to a flat wtructure where all service differentiation is enforced atthe top layer.

R7. 25		/* Number of links (link 0 for empty queue)	*/
Number of distinct links that may be included in the network paths of the flows.
In the LD shaper, this number must be at equal to the number of distinct links that appear
in the network paths of all actual flows, plus 1 unit for the virtual link used by the virtual
empty queue.
In the case of the FLAT shaper, the shaping nodes in the bottom layer of the hierarchy are associated
with the flow queues, not with the network paths of the flows. The number of links in the FLAT case is 0,
because the scheduling algorithm does not take into account the eligibility status of the physical links.

R8. 0	1	0	/* One physical link in single path	*/
First row in the network path table. In each row, the first column is the path identifier
(0 for the path of the virtual empty queue), the second column is the number of links in the
path, the third column is the identifier of the first link in the path (0 for the virtual link
of the virtual empty queue), and the subsequent columns, if present, are the identifiers of
the subsequent links in the network path, in the order they are traversed by packets going
from the source endpoint to the destination endpoint along the path.
If the hierarchical LD shaper is in operation, one row follows the first row for every
distinct network path that may be used by the flows of the source endpoint. Unused network paths
can also be listed, for convenience in the case that flows using those paths are added or
removed in subsequent runs of the program that use the same .inp file. The following example is taken from
row R9 and shows network path #1:
1	3	1	9      17     /* Path 1 to REC-1		*/
Again, the first column is the network path identifier, the second column is the number of physical
links in the path (3), and the next three columns are the identifiers for the three links, each within
the 1:25 range set by the value in row R7. Rows R10-R24 define Network Paths 2-16.

There is no network path table in the .inp file for the FLAT shaper, as a consequence of using
an algorithm that is not augmented with routing awareness (again becasue of the value 0 assigned in row R8).

R25. 1	0.28		0	/* Virtual empty queue has direct entry into root scheduler	*/
First row in the queue table (scheduling nodes served by the bottom layer of the hierarchy).
The first row configures the virtual emnpty queue, to be identified with 0 in the PSS.
In the row, the first column is the number of queues described by the row (always 1 for the
row of the virtual empty queue, the number can be larger for other rows that describe a number
of queues with identical scheduling and routing parameters).
The second column is the rate of each queue in the row, expressed as a fraction of 1.0
(in this example, where 1.0 is 10 Gb/s, 0.0015 is 15 Mb/s). The rate of the virtual empty
queue must always be the 1.0 complement of the sum of the rates of all other queues.
The third column is the identifier of the network path of the queue, matching one of the
identifiers in the previous list of network paths. 

R26. 40	0.0015		5	/* Flows to REC-3 through SP1	*/
Second row in the list of queues. The meanings of the columns are as described for the
row of the virtual empty queue. The rate of the second column is assigned independently.
In this case the network path identifier is 5, pointing to the 6th entry in the network path table
(row R13 in the file).
If different sets of queues differ in one or more of the scheduling rate and network path,
they must be described with respective rows in this portion of the .inp file. Different rows
can also be used for separating queues with identical scheduling and routing parameters
(e.g., this row could be divided into any number of rows between 1 and 40, as long as the
number of queues covered by each row is properly defined).

R27. 40	0.0015		6	/* Flows to REC-3 through SP2	*/
Third row in the list of queues. The same rules apply as for the second row, to this third row
and to all rows that may follow. In the PSS, the queues are identified in the order they
are configured. The virtual empty queue is always 0, the queues in the second row of this
example are numbered from 1 to 40, the queues of the third row are numbered from 41 to 80, and
so on.

The sum of the number of queues in the rows of the queue table must match the value in row R5.

In the .pss output file, each row represents a timeslot of the periodic service sequence.
In each row, the first column is the identifier of the timeslot, the second column is the
identifier of the queue served in the timeslot, the third column (L2: xx) identifies the
network path of the queue, and the subsequent columns (Pyy: zz) identify the links that
compose the network path of the selected queue.

In the FLAT case, the number next to L2: is the same as the queue identifier, because all queues,
including the virtual empty queue, are served through a shaping node that carries the same identifier
as the queue: 28  21  L2: 21

In the LD case, a typical row in the .pss file looks like the following:
28  202  L2: 10  P0: 5  P1: 15  P2: 21,
where 28 is the timeslot identifier, 202 is the queue identifier, 10 is the network path
identifier, 5 is the first link in that path, 15 is the second link, and 21 is the third link.
The path and link identifiers are helpful for characterization of the quality of the PSS, with
respect to regularity in the distribution of service opportunities per interior link.

All scenarios shown in the manuscript have uniform load distribution, with all flows having
the same rate and packet size. The reason is that equal load on all data paths maximizes the
propability for the scenarioto produce larger latencies.

To prove that the computation of the PSS works equally well for flow bandwidth
distributions that are not uniform, we include in the set two .inp files (*-asym.inp) where
each group of flows, associated with a distinct networlk path, has flows with different
service rates. The smoothness of the resulting PSS can be verified by looking at the time
distribution of service opportunities/timeslots assigned to each of the physical links.
