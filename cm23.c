/*
**  File      : cm23.c
**  Notes     : The program simulates a two-layer hierarchical scheduler with configurable
**		combinations of WFQ and WF2Q nodes and possible augmentation of the top layer 
**		with routing awareness.
**
**		All Layer-1 schedulers are work-conserving, the Layer-2 swcheduler is non-work-conserving. 
**		To make the Layer-2 scheduler non-workconserving, the first queue in the list (queue 0) must be 
**		configured with service rate equal to the difference between 1.0 and the sum of
**		the rates allocated to all other queues.
**
**		The single layer-2 scheduling node represents the ingress link of the network.
**		The layer-1 scheduling nodes represent the paths to the egress links.
**
**		The execution of the scheduling algorihm uses the lists of links that compose each
**		network path to constrain the eligibility for service of the paths. Only an eligible path
**		whose links are all eligible for service can be considered for service.
**
**              © 2023 Nokia
**              Licensed under the BSD 3-Clause Clear License
**              SPDX-License-Identifier: BSD-3-Clause-Clear
**
*/

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>

/***************************
**  Constant Definitions  **
***************************/

/* Threshold to decide equality between 'double' values.	*/
#define PRECISION	0.00001

/*  Scheduling Algorithm Identifiers.				*/
#define MINFQ		1
#define WFQ		1
#define WF2Q		2
#define MAXFQ		2

/************************
**  Macro Definitions  **
************************/

#define	max(A, B)	((A) > (B) ? (A) : (B))
#define	min(A, B)	((A) < (B) ? (A) : (B))


/***********************
**  Type Definitions  **
***********************/

typedef struct node_unit	Node;		// Data structure for scheduling nodes
typedef struct phy_node_unit	PhyNode;	// Data structure for physical links

/****************************
**  Structure Definitions  **
****************************/

/* 
**  Structure :	node_unit
**  Remarks   : Node in the scheduling hierarchy
*/
struct node_unit
{
  /* Node parameters and state variables.				*/
  int		node_id;	/* Node identifier (per-layer index).	*/
  double	potential;	/* Node system potential (virtual time).*/
  double	rate;		/* Allocated rate.			*/
  double	SP;		/* Starting Potential.			*/
  double	FP;		/* Finishing Potential.			*/
  double	LFP;		/* Last Finishing Potential.		*/
  Node		*next;		/* Next node in queue.			*/
  /* Uplink node parameters and variables.				*/
  int		ul_id;		/* Identifier of uplink node.		*/
  Node		*ul_ptr;	/* Pointer to the UL node.		*/
  /* Downlink node parameters and variables.				*/
  int		num_dls;	/* Number of supported DL nodes.	*/
  int		num_dlb;	/* Number of backlogged DL nodes.	*/
  double	rate_dlb;	/* Total rate of backlogged DL nodes.	*/
  double        FB_dlb;		/* Total F x r of backlogged DL nodes.	*/
  Node		*dl_head;	/* Pointer to the head of the DL queue.	*/
  /* Node's statistical variables.					*/
  long		serv_pack;	/* Total number of services.		*/
  /* List of physical nodes (links) associated with the node (path).	*/
  int		no_phy_nodes;	/* Number of nodes in the list.		*/
  int		*phy_node_id;	/* Array of physical node identifiers.	*/
};


/* 
**  Structure :	phy_node_unit
**  Remarks   : Physical node, mapping onto a single topology node.
*/
struct phy_node_unit
{
  int		node_id;	/* Node identifier (per-layer index).	*/
  double	rate;		/* Allocated rate.			*/
  double	SP;		/* Starting Potential.			*/
  double	FP;		/* Finishing Potential.			*/
};


/***********************
**  Global Variables  **
***********************/

char		*fgetsreturn;	/* Return value of fgets function.	*/
char		pssname[40];	/* Periodic service sequence file name.	*/
char		srvname[40];	/* Services file name.			*/

int		fq_L1;		/* L1 scheduler type.			*/
int		fq_L2;		/* L2 scheduler type.			*/

int		no_L1;		/* Number of L1 nodes (queues).		*/
int		no_L2;		/* Number of L2 nodes (paths).		*/

int		no_Phy;		/* Number of physical nodes.		*/

long		no_slot;	/* Maximum simulation time.		*/
long		curr_time;	/* Current simulation time.		*/

Node		*L1_vect;	/* Array of L1 nodes.			*/
Node		*L2_vect;	/* Array of L2 nodes.			*/

Node		*L1_next_tr;	/* Next L1 node to be served.		*/
Node		*L2_next_tr;	/* Next L2 node to be served.		*/

PhyNode		*Phy_vect;	/* Array of physical nodes.		*/

Node		out_port;	/* Multiplexer to the output port.	*/

FILE		*fpss;		/* File with periodic service sequence.	*/
FILE		*fsrv;		/* File collecting total services.	*/

/************************
**  Function Pointers  **
************************/

Node*	(*select_next_L1)(Node *);
Node*	(*select_next_L2)(Node *);


/**************************
**  Function Prototypes  **
**************************/

void		enqueue_node(Node *, Node *);

void		parinsert(void);

void		print_pss(FILE *);
void		print_srv(FILE *);

void		init_nodes(void);
void		print_node(Node *);

void		serve_queue(void);

Node		*wfq_select(Node *);
Node		*wf2q_select(Node *);

int		is_eligible(Node *);

/***************************
**  Function Definitions  **
***************************/

/*
**  Utility functions
*/

/*
**  Node handling functions
*/

/*
**  Function  :	void enqueue_node(Node *ul_node, Node *in_node)
**  Remarks   : in_node is queued at ul_node.
*/
void enqueue_node(Node *ul_node, Node *in_node)
{
  int		looking = 1;
  Node		*aux_node = ul_node->dl_head;
  Node		*prv_node = NULL;

  in_node->next = NULL;

  if(aux_node == NULL)
    {
      /* The UL node is empty: the arriving node is placed at the head	*/
      /* of the queue.							*/
      ul_node->dl_head = in_node;
    }
  else if(in_node->FP < aux_node->FP)
    {
      /* The queue is not empty.					*/
      /* The arriving node is placed at the head of the queue.		*/
      in_node->next = aux_node;
      ul_node->dl_head = in_node;
    }
  else if((in_node->FP == aux_node->FP) && (in_node->rate > aux_node->rate))
    {
      /* The queue is not empty.					*/
      /* The arriving node is placed at the head of the queue.		*/
      in_node->next = aux_node;
      ul_node->dl_head = in_node;
    }
  else
    {
      /* The queue is not empty.						*/
      /* The arriving node is not placed at the head of the queue.		*/
      prv_node = aux_node;
      aux_node = aux_node->next;
      while((aux_node != NULL) && (looking))
	{
	  if(in_node->FP < aux_node->FP)
	    {
	      looking = 0;
	    }
	  else if((in_node->FP == aux_node->FP) && (in_node->rate > aux_node->rate))
	    {
	      looking = 0;
	    }
	  else
	    {
	      prv_node = aux_node;
	      aux_node = aux_node->next;
	    }
	}
      prv_node->next = in_node;
      in_node->next = aux_node;
    }

  return;
}


/*
**  Input Functions
*/

/*
**  Function  : void parinsert(void)
**  Remarks   : the system parameters are assigned here.
*/
void parinsert(void)
{  
  int		ii;
  long		ll;
  int		curr_no_L1, loc_no_L1;
  int		curr_no_L2, loc_no_L2;
  int		loc_path_length;
  int		L2idx;
  int		phy_id;
  char		ptr[40];
  char		comment[200];
  char		partstr[200];
  char		ret[200];
  double	tot_rate;

  tot_rate = 0.0;
  no_L1 = no_L2 = 0;

  // Output File Radix	
  do
    {
      //      printf("\nEnter the file radix				: ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%s", ptr);
    } while(comment[0] == '\0');
  printf("\nFile radix: %s\n", ptr);
  
  strcpy(pssname, ptr);
  strcat(pssname, ".pss"); 

  strcpy(srvname, ptr);
  strcat(srvname, ".srv");
  
  //  L1 Scheduling Algorithm
  do
    {
      //      printf("\n(1)  WFQ  (Weighted Fair Queueing)");
      //      printf("\n(2)  WF2Q (Worst-case Fair Weighted Fair Queueing)");
      //      printf("\nEnter your choice : ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%d", &fq_L1);
    } while((fq_L1 < MINFQ) || (fq_L1 > MAXFQ) || (comment[0] == '\0'));
  switch(fq_L1)
    {
    case 1:
      printf("\nL1 Scheduler: WFQ\n");
      break;

    case 2:
      printf("\nL1 Scheduler: WF2Q\n");
      break;

    default:
      printf("\nError: L1 Scheduler Not Recognized (%d)\n", fq_L1);
      exit(1);
    }

  // L2 Scheduling Algorithm
  do
    {
      //  printf("\n(1)  WFQ  (Weighted Fair Queueing)");
      //      printf("\n(2)  WF2Q (Worst-case Fair Weighted Fair Queueing)");
      //      printf("\nEnter your choice : ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%d", &fq_L2);
    } while((fq_L2 < MINFQ) || (fq_L2 > MAXFQ) || (comment[0] == '\0'));
  switch(fq_L2)
    {
    case 1:
      printf("\nL2 Scheduler: WFQ\n");
      break;

    case 2:
      printf("\nL2 Scheduler: WF2Q\n");
      break;

    default:
      printf("\nError: L2 Scheduler Not Recognized (%d)\n", fq_L2);
      exit(1);
    }

  // Number of Slots
  do
    {
      //      printf("\nNumber of time slots				: ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%ld", &no_slot);
    } while((no_slot < 1) || (comment[0] == '\0'));
  printf("\nNumber of time slots: %ld\n", no_slot);

  // Number of L1 scheduling nodes (queues)
  do
    {
      //      printf("\nFirst Level. Enter the number of nodes (queues)	: ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%d", &no_L1);
    } while((no_L1 < 1) || (comment[0] == '\0'));
  printf("\nFirst Level. Number of nodes (queues): %d\n", no_L1);

  // Number of L2 scheduling nodes (paths)
  do
    {
      //      printf("\nSecond Level. Enter the number of nodes (paths)	: ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%d", &no_L2);
    } while((no_L2 < 1) || (comment[0] == '\0'));
  printf("\nSecond Level. Number of nodes (paths): %d\n", no_L2);

  L1_vect = (Node *)malloc(no_L1 * sizeof(Node));
  L2_vect = (Node *)malloc(no_L2 * sizeof(Node));

  // Number of physical links
  do
    {
      //      printf("\nEnter the number of physical links	 	: ");
      fgetsreturn = fgets(comment, 200, stdin);
      sscanf(comment, "%d", &no_Phy);
    } while((no_Phy < 0) || (comment[0] == '\0'));
  printf("\nNumber of physical links: %d\n", no_Phy);

  if (no_Phy > 0)
    {
      Phy_vect = (PhyNode *)malloc(no_Phy * sizeof(PhyNode));
    }
  
  /* Generic initialization of all nodes				*/
  init_nodes();

  if (no_Phy > 0)
    {
      curr_no_L2 = 0;
      printf("\nPath definition: PathID - PathLength - LinkID 1 - LinkID2 - ...\n");
      while (curr_no_L2 < no_L2)
	{
	  do
	    {
	      fgetsreturn = fgets(comment, 200, stdin);
	      sscanf(comment, "%d %d %200c", 
		     &loc_no_L2,
		     &loc_path_length,
		     partstr);
	    } while((comment[0] == '\0') || (comment[0] == '\n'));
	  // Sanity check on path index: no double allocation is allowed!
	  if (loc_no_L2 != curr_no_L2)
	    {
	      printf("\nWrong path identifier (%d) - Should be %d instead!\n",
		     loc_no_L2, curr_no_L2);
	    }
	  if (loc_path_length < 1)
	    {
	      printf("\nWrong number of links in path (%d)\n",
		     loc_path_length);
	    }
	  // Setting up the array of link identifiers
	  L2_vect[curr_no_L2].no_phy_nodes = loc_path_length;
	  L2_vect[curr_no_L2].phy_node_id = (int *)malloc(loc_path_length * sizeof(int));
	  
	  for (ll = 0; ll < loc_path_length; ll++)
	    {
	      sscanf(partstr, "%d %200c", &(L2_vect[curr_no_L2].phy_node_id[ll]), ret);
	      strcpy(partstr, ret);
	    }	      
	  curr_no_L2++;
	} 
    }
  
  curr_no_L1 = 0;

  printf("\nDefinition of queue parameters: #Queues Tot - #Queues in Line - Rate - L2\n");
  while(curr_no_L1 < no_L1)
    {
      do
	{
	  fgetsreturn = fgets(comment, 200, stdin);
	  sscanf(comment, "%d %lf %d", 
		 &loc_no_L1, 
		 &(L1_vect[curr_no_L1].rate), 
		 &L2idx);
	} while((comment[0] == '\0') || (comment[0] == '\n'));

      // Sanity check on accumulated number of queues
      if((loc_no_L1 <= 0) || ((curr_no_L1 + loc_no_L1) > no_L1))
	{
	  printf("\nWrong Number of queues !!\n");
	  exit(1);
	} 
      
      tot_rate += (loc_no_L1 * L1_vect[curr_no_L1].rate);
      if((L1_vect[curr_no_L1].rate < 0.0) || (tot_rate > (1.0 + PRECISION)))
	{
	  printf("\nWrong Allocated Rate Assignment: %.6f !!\n",
		 tot_rate);
	  exit(1);
	} 

      if((L2idx < 0) || (L2idx >= no_L2))
	{
	  printf("\nL2 ID is out of range!!\n");
	  exit(1);
	} 

      /* Applying the line values to all queues it covers.		*/
      for (ii = 0; ii < loc_no_L1; ii++)
	{
	  L1_vect[curr_no_L1 + ii].rate = L1_vect[curr_no_L1].rate; 
	  L1_vect[curr_no_L1 + ii].ul_id = L2idx; 
	  L1_vect[curr_no_L1 + ii].ul_ptr = &(L2_vect[L2idx]); 
	  L2_vect[L2idx].num_dls++;
	  L2_vect[L2idx].rate += L1_vect[curr_no_L1].rate;
	  if (no_Phy > 0)
	    {
	      for (ll = 0; ll < L2_vect[L2idx].no_phy_nodes; ll++)
		{
		  Phy_vect[L2_vect[L2idx].phy_node_id[ll]].rate += L1_vect[curr_no_L1].rate;
		}
	    }
	}
      curr_no_L1 += loc_no_L1;
    }

  if(curr_no_L1 < no_L1)
    {
      printf("\nWarning !!");
      printf("\nInitially declared VCs : %d\t\tActually Defined VCs : %d",
	     no_L1, curr_no_L1);
      no_L1 = curr_no_L1;
    }

  printf("\nThe total allocated rate is : %f", tot_rate);
  
  /* Number of supported DL nodes in the output port.	*/
  for(ii = 0; ii < no_L2; ii++)
    {
      if (L2_vect[ii].num_dls > 0)
	{
	  out_port.num_dls++;
	  out_port.rate += L2_vect[ii].rate;
	}
    }
}


/*
**  Output Functions
*/

/*
**  Function  :	void print_pss(FILE *fseq)
**  Remarks   : Print the allocated service in the file pointed by 'fseq'.
*/
void print_pss(FILE *fseq)
{
  int jj;
  if(L1_next_tr != NULL)
    {
      fprintf(fseq, "%ld  %d  L2: %d", curr_time - 1,
	      L1_next_tr->node_id,
	      L2_next_tr->node_id);
      if (no_Phy > 0)
	{
	  for(jj = 0; jj < L2_next_tr->no_phy_nodes; jj++)
	    {
	      fprintf(fseq, "  P%d: %d", jj, L2_next_tr->phy_node_id[jj]);
	    }
	}
      fprintf(fseq, "\n");
    }
  else
    {
      fprintf(fseq, "%ld  -1  L2: -1\n", curr_time - 1);
    }
}


/*
**  Function  :	void print_srv(FILE *fp)
**  Remarks   : the numbers of per-queue services are printed in the
**		file pointed by 'fp'.
*/
void print_srv(FILE *fp)
{
  int	cc;
  long	tot_srv = 0;

  for(cc = 0; cc < no_L1; cc++)
    {
      fprintf(fp, "\n%d\t\t%.6f\t%ld ", cc,
	      L1_vect[cc].rate, L1_vect[cc].serv_pack);
      tot_srv += L1_vect[cc].serv_pack;
    }
  fprintf(fp, "\n\nTotal Slots : %ld", tot_srv);
}


/*
**  Function  :	void init_nodes(void)
**  Remarks   :	generic initialization of all scheduling nodes.
*/
void init_nodes(void)
{
  int		ii;

  /* Generic initialization of L1 nodes.		*/
  for(ii = 0; ii < no_L1; ii++)
    {
      L1_vect[ii].node_id = ii;
      L1_vect[ii].potential = 0.0;
      L1_vect[ii].rate = 0.0;
      L1_vect[ii].SP = 0.0;
      L1_vect[ii].FP = 0.0;
      L1_vect[ii].LFP = 0.0;
      L1_vect[ii].next = NULL;
      L1_vect[ii].ul_id = -1;
      L1_vect[ii].ul_ptr = NULL;
      L1_vect[ii].num_dls = 0;
      L1_vect[ii].num_dlb = 0;
      L1_vect[ii].rate_dlb = 0.0;
      L1_vect[ii].FB_dlb = 0.0;
      L1_vect[ii].dl_head = NULL;
      L1_vect[ii].serv_pack = 0;
      L1_vect[ii].no_phy_nodes = 0;
      L1_vect[ii].phy_node_id = NULL;
    }
  /* Generic initialization of L2 nodes.		*/
  for(ii = 0; ii < no_L2; ii++)
    {
      L2_vect[ii].node_id = ii;
      L2_vect[ii].potential = 0.0;
      L2_vect[ii].rate = 0.0;
      L2_vect[ii].SP = 0.0;
      L2_vect[ii].FP = 0.0;
      L2_vect[ii].LFP = 0.0;
      L2_vect[ii].next = NULL;
      L2_vect[ii].ul_id = 0;
      L2_vect[ii].ul_ptr = &(out_port);
      L2_vect[ii].num_dls = 0;
      L2_vect[ii].num_dlb = 0;
      L2_vect[ii].rate_dlb = 0.0;
      L2_vect[ii].FB_dlb = 0.0;
      L2_vect[ii].dl_head = NULL;
      L2_vect[ii].serv_pack = 0;
      L2_vect[ii].no_phy_nodes = 0;
      L2_vect[ii].phy_node_id = NULL;
    }
  /* Generic initialization of physical links.		*/
  for(ii = 0; ii < no_Phy; ii++)
    {
      Phy_vect[ii].node_id = ii;
      Phy_vect[ii].rate = 0.0;
      Phy_vect[ii].SP = 0.0;
      Phy_vect[ii].FP = 0.0;
    }
}


/*
**  Function  :	void print_node(Node *)
**  Remarks   :	print all the variables for the node of the pointer.
*/
void print_node(Node *gn)
{
  int		ii;

  printf("id rate ul_id ul_ptr num_dls num_dlb rate_dlb FB_dlb NoPhyNodes\n");
  printf("%d %f %d %p %d %d %f %f %d\n",
	 gn->node_id,
	 gn->rate,
	 gn->ul_id,
	 gn->ul_ptr,
	 gn->num_dls,
	 gn->num_dlb,
	 gn->rate_dlb,
	 gn->FB_dlb,
	 gn->no_phy_nodes
	 );
}


/*
**  Function  :	void load_queues(void)
**  Remarks   :	load all queues and all respective hierarchical nodes
**		with an unlimited number of packets.
*/
void load_queues(void)
{
  int		ii, cc;

  /* The arriving packets reach their session queues.			*/
  for(cc = 0; cc < no_L1; cc++)
    {
      /* A new packet arrives to each queue.				*/
      Node	*L1node = L1_vect + cc;
      Node	*L2node = L1node->ul_ptr;

      if(L1node->rate > 0.0)
	{
	  /* Configured queue with non-zero rate allocation.		*/
	  
	  /* Updating the queue attributes.				*/
	  L1node->SP = max(L1node->LFP, L2node->potential);
	  L1node->FP = L1node->SP + (1.0 / L1node->rate);
	  
	  /* Enqueueing the queue.					*/
	  enqueue_node(L2node, L1node);
	  
	  /* Checking the state of the L2node of the queue.		*/
	  if(L2node->num_dlb == 0)
	    {
	      /* Newly Backlogged L2node.				*/
	      
	      /* Setting the attributes of the L2node.			*/
	      L2node->SP = max(L2node->LFP, out_port.potential);
	      L2node->FP = L2node->SP + (1.0 / L2node->rate);
	      
	      enqueue_node(&out_port, L2node);
	      
	      /* Updating the potential variables for the port.	*/
	      out_port.num_dlb++;
	      out_port.rate_dlb += L2node->rate;
	      out_port.FB_dlb += (L2node->FP * L2node->rate);
	    }

	  /* Updating the potential variables of the L2 node.		*/
	  L2node->num_dlb++;
	  L2node->rate_dlb += L1node->rate;
	  L2node->FB_dlb += (L1node->FP * L1node->rate);
	}
    }
  
  /* Loading the physical nodes.					*/
  for (ii = 0; ii < no_Phy; ii++)
    {
      if(Phy_vect[ii].rate > 0.0)
	{
	  Phy_vect[ii].SP = out_port.potential;
	  Phy_vect[ii].FP = Phy_vect[ii].SP + (1 / Phy_vect[ii].rate);
	}
      else
	{
	  Phy_vect[ii].SP = 0.0;
	  Phy_vect[ii].FP = 0.0;
	}	  
    }
}


/*
**  Function  :	void serve_queue(void)
**  Remarks   :	the queue selected during the previous scheduling 
**		round is "served" here.
*/
void serve_queue(void)
{
  int		cc, ii;
  long		delay;
  Node		*auxnode;
  Node		*prevnode;

  if(L1_next_tr == NULL)
    {
      /* No queue to serve: nothing to be done.			*/
      return;
    }
  
  /* Extract the serviced queue from its L2 queue.		*/
  for(auxnode = L2_next_tr->dl_head;
      (auxnode != NULL) && (auxnode != L1_next_tr);
      prevnode = auxnode, auxnode = auxnode->next);
  if(auxnode == NULL)
    {
      printf("\nError: Queue selected for service was not found!\n");
      exit(1);
    }
  else
    {
      if(auxnode == L2_next_tr->dl_head)
	{
	  L2_next_tr->dl_head = auxnode->next;
	  auxnode->next = NULL;
	}
      else
	{
	  prevnode->next = auxnode->next;
	  auxnode->next = NULL;
	}
    }
  
  /* Extract the serviced L2 node from the output port queue.	*/
  for(auxnode = out_port.dl_head;
      (auxnode != NULL) && (auxnode != L2_next_tr);
      prevnode = auxnode, auxnode = auxnode->next);
  if(auxnode == NULL)
    {
      printf("\nError: L2 node selected for service was not found!\n");
      exit(1);
    }
  else
    {
      if(auxnode == out_port.dl_head)
	{
	  out_port.dl_head = auxnode->next;
	  auxnode->next = NULL;
	}
      else
	{
	  prevnode->next = auxnode->next;
	  auxnode->next = NULL;
	}
    }

  /* Updating the service count for the serviced queue.		*/
  L1_next_tr->serv_pack++;

  /* Updating one variable for calculation of the potential at	*/
  /* all levels, independently of the state of the queue.	*/
  L2_next_tr->FB_dlb -= (L1_next_tr->FP * L1_next_tr->rate);
  out_port.FB_dlb -= (L2_next_tr->FP * L2_next_tr->rate);

  /* The serviced queue (L1 node) always remains backlogged.	*/
  /*  L1_next_tr->SP = max(L1_next_tr->FP, L2_next_tr->potential);*/
  L1_next_tr->SP = L1_next_tr->FP;
  L1_next_tr->FP = L1_next_tr->SP + (1.0 / L1_next_tr->rate);
      
  enqueue_node(L2_next_tr, L1_next_tr);
      
  L2_next_tr->FB_dlb += (L1_next_tr->FP * L1_next_tr->rate);
      
  L2_next_tr->potential = 
    max(L2_next_tr->potential + 1.0, 
	(L2_next_tr->FB_dlb - L2_next_tr->num_dlb) / L2_next_tr->rate_dlb);
      
  /* The serviced L2 node is always backlogged.			*/
  /*  L2_next_tr->SP = max(L2_next_tr->FP, out_port.potential); */
  L2_next_tr->SP = L2_next_tr->FP;
  L2_next_tr->FP = L2_next_tr->SP + (1.0 / L2_next_tr->rate);

  enqueue_node(&out_port, L2_next_tr);

  /* Update the state of the physical nodes.			*/
  if (no_Phy > 0)
    {
      for (ii = 0; ii < L2_next_tr->no_phy_nodes; ii++)
	{
	  Phy_vect[L2_next_tr->phy_node_id[ii]].SP = Phy_vect[L2_next_tr->phy_node_id[ii]].FP;
	  Phy_vect[L2_next_tr->phy_node_id[ii]].FP = Phy_vect[L2_next_tr->phy_node_id[ii]].SP +
	    (1.0 / Phy_vect[L2_next_tr->phy_node_id[ii]].rate);
	}
    }

  /* Update the state of the output port.			*/
  out_port.FB_dlb += (L2_next_tr->FP * L2_next_tr->rate);
      
  out_port.potential = 
    max(out_port.potential + 1.0, 
	(out_port.FB_dlb - out_port.num_dlb) / out_port.rate_dlb);
}


/*
**  Function  :	Node *wfq_select(Node *sched)
**  Remarks   :	the next node to be serviced in the WFQ scheduler of the node 
**		passed as argument.
*/
Node *wfq_select(Node *sched)
{
  Node *auxnode;

  if(sched == NULL)
    {
      return NULL;
    }
 
  if(sched->num_dlb == 0)
    {
      /* No backlogged DL node available for scheduling.	*/
      return NULL;
    }

  auxnode = sched->dl_head;
  if(auxnode == NULL)
    {
      /* No head DL node found despite positive counter for	*/
      /* backlogged nodes.					*/
      printf("\nError: NULL head pointer in scheduler with %d backlogged nodes !!\n",
	     sched->num_dlb);
      exit(1);
    }

  /* Only return the pointer to the selected node.		*/
  return(auxnode);
}


/*
**  Function  :	Node *wf2q_select(Node *sched)
**  Remarks   :	the next node to be serviced in the WF2Q scheduler of the node 
**		passed as argument.
*/
Node *wf2q_select(Node *sched)
{
  Node *auxnode;
  Node *frstchoice = NULL;
  Node *scndchoice = NULL;
  int ii;
  
  if(sched == NULL)
    {
      return NULL;
    }
 
  if(sched->num_dlb == 0)
    {
      /* No backlogged DL node available for scheduling.	*/
      return NULL;
    }

  auxnode = sched->dl_head;
  while((auxnode != NULL) && (frstchoice == NULL))
    {
      // Description of the scheduling algorithm:
      // The scheduler scans the list of queued nodes to find the next node to serve. First choice is the eligible node
      // with minimum timestamp where all physical links are eligible for service. Second choice is the eligible node
      // with minimum timestamp, to be used in case no node with all physical links eligible can be found.
      if(auxnode->SP <= (sched->potential + PRECISION))
	{
	  /* Found second-choice node.			*/
	  if(scndchoice == NULL)
	    {
	      scndchoice = auxnode;
	    }

	  if (auxnode->no_phy_nodes == 0)
	    {
	      frstchoice = auxnode;
	    }
	  else
	    {
	      for (ii = 0; ii < auxnode->no_phy_nodes; ii++)
		{
		  if (Phy_vect[auxnode->phy_node_id[ii]].SP > (out_port.potential + PRECISION))
		    {
		      printf("\nTime %ld  Found non-eligible link %d for path %d SP %f Potential %f, %d physical nodes)",
			     curr_time, auxnode->phy_node_id[ii], auxnode->node_id, auxnode->SP, sched->potential,
			     auxnode->no_phy_nodes);
		      
		      break;
		    }
		}
	      if (ii == auxnode->no_phy_nodes)
		{
		  // The 'for' cycle did not break: all links in the path are eligible for service
		  frstchoice = auxnode;

		  printf("\nTime %ld  Found FirstChoice %p (%d) SP %f FP %f Potential %f Delta: %.2f, %d physical nodes)",
			 curr_time, auxnode, auxnode->node_id, auxnode->SP, auxnode->FP, sched->potential,
			 (auxnode->FP - sched->potential) * auxnode->rate, auxnode->no_phy_nodes);
		  
		}
	    }
	}

      /* Keep looking.	*/		
      auxnode = auxnode->next;
    }
  
  if (frstchoice != NULL)
    {
      /* Return first choice node if found.			*/
      return (frstchoice);
    }
  
  /* Return second choice if no first.				*/
  return (scndchoice);
}


/*
**  Main Function
*/

/*
**  Function  :	int main(void)
*/ 
int main(void)
{
  int	cc;

  parinsert();

  switch(fq_L1)
    {
    case WFQ :
      select_next_L1 = wfq_select;
      break;

    case WF2Q :
      select_next_L1 = wf2q_select;
      break;

    default :
      printf("\nError in selection of L1 (Queue Level) Scheduling Algorithm !!\n");
      exit(1);
    }

  switch(fq_L2)
    {
    case WFQ :
      select_next_L2 = wfq_select;
      break;
      
    case WF2Q :
      select_next_L2 = wf2q_select;
      break;

    default :
      printf("\nError in selection of L2 Scheduling Algorithm !!\n");
      exit(1);
    }

  if((fsrv = fopen(srvname, "w")) == NULL)
    {
      printf("\nError opening the '.srv' data file.\n");
      exit(1);
    }
  fprintf(fsrv, "\nQueue\tServices\tRate\t\tSlots");

  if((fpss = fopen(pssname, "w")) == NULL)
    {
      printf("\nError opening the '.pss' data file.\n");
      exit(1);
    }
  
  /* Initializing the output port scheduler.				*/
  out_port.num_dlb = 0;
  out_port.rate_dlb = 0.0;
  out_port.FB_dlb = 0.0;
  out_port.potential = 0.0;

  /* Increment the queued packet counters.				*/
  load_queues();

  /* Automatic session: the simulation is stopped when the maximum	*/
  /* number of slots is reached.					*/
  for(curr_time = 0; curr_time < no_slot; curr_time++)
    {
      if((curr_time > 0) && (curr_time <= no_slot))
	{
	  /* Updating the periodic service sequence.			*/
	  print_pss(fpss);
	}

      /* Serve the queue selected in the previous scheduling round.	*/
      serve_queue();
      /* Select the queue to receive the next service.			*/
      L2_next_tr = select_next_L2(&out_port);
      L1_next_tr = select_next_L1(L2_next_tr);
    }    

  /* Printing the number of services received by each flow.		*/
  print_srv(fsrv);
  fprintf(fsrv, "\n");
  fclose(fsrv);
  fclose(fpss);
  
  printf("\n");

  return 0;
}
