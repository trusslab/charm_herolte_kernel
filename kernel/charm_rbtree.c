/* Copyright (C) 2016-2018 University of California, Irvine
 * 
 * Authors:
 * Seyed Mohammadjavad Seyed Talebi <mjavad@uci.edu>
 * Ardalan Amiri Sani <arrdalan@gmail.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/rbtree.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>

struct unode{
       struct rb_node __rb_node;
       uint64_t __num;      //this is to record our data.
};

// this function is implemented based on rb_tree implementation on :
//       http://chrisincs.blogspot.com/2011/05/introduction-to-linux-red-black-tree.html
struct unode * rb_search_unode( struct rb_root * root , uint64_t target ){

       struct rb_node * n = root->rb_node;
       struct unode * ans;

       while( n ){
              //Get the parent struct to obtain the data for comparison
              ans = rb_entry( n , struct unode , __rb_node );

              if( target < ans->__num )
                     n = n->rb_left;
              else if( target > ans->__num )
                     n = n->rb_right;
              else
                     return ans;

       }
       return NULL;

}
// this function is implemented based on rb_tree implementation on :
//       http://chrisincs.blogspot.com/2011/05/introduction-to-linux-red-black-tree.html
struct unode * rb_insert_unode( struct rb_root * root , uint64_t target , struct rb_node * source ){

       struct rb_node **p = &root->rb_node;
       struct rb_node *parent = NULL;
       struct unode * ans;

       while( *p ){

              parent = *p;
              ans = rb_entry( parent , struct unode , __rb_node );

              if( target < ans->__num )
                     p = &(*p)->rb_left;
              else if( target > ans->__num )
                     p = &(*p)->rb_right;
              else
                     return ans;

       }
       rb_link_node( source , parent , p );             //Insert this new node as a red leaf.
       rb_insert_color( source , root );           //Rebalance the tree, finish inserting
       return NULL;

}

// this function is implemented based on rb_tree implementation on :
//       http://chrisincs.blogspot.com/2011/05/introduction-to-linux-red-black-tree.html
void rb_erase_unode( struct rb_node * source , struct rb_root * root ){

       struct unode * target;
      
       target = rb_entry( source , struct unode , __rb_node );
       rb_erase( source , root );                           //Erase the node
       kfree( target );                                     //Free the memory

}

struct rb_root pointers_rb_root = RB_ROOT;
//------------------using rb_tree for chekcing valid pointers
void add_valid_pointer(uint64_t ptr)
{
	struct unode * node;	
	struct unode * found_node=NULL;
	node = ( struct unode * )kmalloc( sizeof( struct unode ),GFP_KERNEL);
	found_node = rb_insert_unode( &pointers_rb_root, ptr , &node->__rb_node ); 
	if(found_node==NULL){
		node->__num = ptr;	
	}else{
		kfree( node );
	}
}

void remove_valid_pointer(uint64_t ptr){
	
	struct unode * found_node=NULL;
	found_node=rb_search_unode( &pointers_rb_root , ptr );
	if(found_node!=NULL){
		rb_erase_unode( &found_node->__rb_node , &pointers_rb_root );
	}
}

int pointer_is_valid(uint64_t ptr)
{
	struct unode * found_node=NULL;
	found_node=rb_search_unode( &pointers_rb_root , ptr );
	if(found_node==NULL){
		return 0;
	}else{
		return 1;
	}
}
