/* Masstree
 * Eddie Kohler, Yandong Mao, Robert Morris
 * Copyright (c) 2012-2013 President and Fellows of Harvard College
 * Copyright-2013 (c) 2012 Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, subject to the conditions
 * listed in the Masstree LICENSE file. These conditions include: you must
 * preserve this copyright notice, and you cannot mention the copyright
 * holders in advertising related to the Software without their permission.
 * The Software is provided WITHOUT ANY WARRANTY, EXPRESS OR IMPLIED. This
 * notice is a summary of the Masstree LICENSE file; the license in that file
 * is legally binding.
 */
#ifndef MASSTREE_GET_HH
#define MASSTREE_GET_HH 1
#include "masstree_tcursor.hh"
#include "masstree_key.hh"
#include <deque>
#include <map>
#include <iostream>
#include <fstream>

namespace Masstree {

template <typename P>
inline int unlocked_tcursor<P>::lower_bound_binary() const
{
    int l = 0, r = perm_.size();
    while (l < r) {
        int m = (l + r) >> 1;
        int mp = perm_[m];
        int cmp = key_compare(ka_, *n_, mp);
        if (cmp < 0)
            r = m;
        else if (cmp == 0)
            return mp;
        else
            l = m + 1;
    }
    return -1;
}

template <typename P>
inline int unlocked_tcursor<P>::lower_bound_linear() const
{
    int l = 0, r = perm_.size();
    while (l < r) {
        int lp = perm_[l];
        int cmp = key_compare(ka_, *n_, lp);
        if (cmp < 0)
            break;
        else if (cmp == 0)
            return lp;
        else
            ++l;
    }
    return -1;
}

/*
	hyw:
	This method is a sample to get number of keys
	in each masstree node
*/

template<typename P>
void unlocked_tcursor<P>::keyCountsPerMass() {	
    std::ofstream myfile;
    myfile.open("code_output.txt");
	std::deque <leafvalue<P> > q;
	std::map<int, int>  myMap;
	int kp, keylenx = 0;
	int l1 = 0;
    int l2 = 0;
    int total_mass = 0;
    int total_keys = 0;
    int max_mass_level = 0;
    int keysPerMass = 0;
	node_base<P>* root = const_cast<node_base<P>*>(root_);
	leaf<P> *next;

 nextMass:
    //myfile<<"{";
    total_mass += 1;
 	n_ = root->leftmost();
 nextNeighbor:
    n_->prefetch();
 	perm_ = n_->permutation();
 	//myfile<< perm_.size() << ", ";
    total_keys += perm_.size(); 
    keysPerMass += perm_.size();
 	for(int i = 0 ; i < perm_.size(); i++) {
 		kp = perm_[i];
 		keylenx = n_->keylenx_[kp];
 		if (n_->keylenx_is_layer(keylenx)) {
 			q.push_back(n_->lv_[kp]);
 			l1++;
 		}
 	}
 	if((next = n_->safe_next())) {
 		n_ = next;
 		goto nextNeighbor;
 	} 

 	if(q.size() != 0) {
        //myfile<<"} ";
 	    if (l2 == 0) {
            max_mass_level += 1;
 		    l2 = l1;
 		    l1 = 0;
 			//myfile<<"\n";
 	    }
 		root = q.front().layer();
        q.pop_front();
 		--l2; 

 		if(myMap.find(keysPerMass) != myMap.end()){
          	myMap[keysPerMass] += 1;
      		} else {
          	myMap[keysPerMass] = 1;
      	}
      	keysPerMass = 0;
 		goto nextMass;
 	}

    if(myMap.find(keysPerMass) != myMap.end()){
      myMap[keysPerMass] += 1;
    } else {
      myMap[keysPerMass] = 1;
    }

    myfile<<"}\ntotalMassN: " << total_mass << "\ntotalK: "<< total_keys <<"\navg: "<<(float)total_keys/total_mass<<"\nmaxMassLevel: "<<max_mass_level<<"\n";
    for(std::map<int, int>:: iterator it= myMap.begin(); it != myMap.end(); ++it)
    	myfile<< it->first <<"\n";
    myfile<< "---------------------------------------------------------------------------\n";
    for(std::map<int, int>:: iterator it= myMap.begin(); it != myMap.end(); ++it)
    	myfile<< it->second<<"\n";
    myfile.close();
}

template <typename P>
bool unlocked_tcursor<P>::find_unlocked(threadinfo& ti)
{
    bool ksuf_match = false;
    int kp, keylenx = 0;
    node_base<P>* root = const_cast<node_base<P>*>(root_);

 retry:
    n_ = root->reach_leaf(ka_, v_, ti);

 forward:
    if (v_.deleted())
	goto retry;

    n_->prefetch();
    perm_ = n_->permutation();
    if (leaf<P>::bound_type::is_binary)
        kp = lower_bound_binary();
    else
        kp = lower_bound_linear();
    if (kp >= 0) {
	keylenx = n_->keylenx_[kp];
	fence();		// see note in check_leaf_insert()
	lv_ = n_->lv_[kp];
	lv_.prefetch(keylenx);
	ksuf_match = n_->ksuf_equals(kp, ka_, keylenx);
    }
    if (n_->has_changed(v_)) {
	ti.mark(threadcounter(tc_stable_leaf_insert + n_->simple_has_split(v_)));
	n_ = n_->advance_to_key(ka_, v_, ti);
	goto forward;
    }

    if (kp < 0)
	return false;
    else if (n_->keylenx_is_layer(keylenx)) {
	if (likely(n_->keylenx_is_stable_layer(keylenx))) {
	    ka_.shift();
	    root = lv_.layer();
	    goto retry;
	} else
	    goto forward;
    } else
        return ksuf_match;
}

template <typename P>
inline bool basic_table<P>::get(Str key, value_type &value,
                                threadinfo& ti) const
{
    unlocked_tcursor<P> lp(*this, key);
    bool found = lp.find_unlocked(ti);
    if (found)
	value = lp.value();
    return found;
}

template <typename P>
inline node_base<P>* tcursor<P>::get_leaf_locked(node_type* root,
                                                 nodeversion_type& v,
                                                 threadinfo& ti)
{
    nodeversion_type oldv = v;
    typename permuter_type::storage_type old_perm;
    leaf_type *next;

    n_->prefetch();

    if (!ka_.has_suffix())
	v = n_->lock(oldv, ti.lock_fence(tc_leaf_lock));
    else {
	// First, look up without locking.
	// The goal is to avoid dirtying cache lines on upper layers of a long
	// key walk. But we do lock if the next layer has split.
	old_perm = n_->permutation_;
	ki_ = leaf_type::bound_type::lower_with_position(ka_, *n_, kp_);
	if (kp_ >= 0 && n_->value_is_stable_layer(kp_)) {
	    fence();
	    leafvalue_type entry(n_->lv_[kp_]);
	    entry.layer()->prefetch_full();
	    fence();
	    if (likely(!v.deleted()) && !n_->has_changed(oldv, old_perm)
		&& !entry.layer()->has_split()) {
		ka_.shift();
		return entry.layer();
	    }
	}

	// Otherwise lock.
	v = n_->lock(oldv, ti.lock_fence(tc_leaf_lock));

	// Maybe the old position works.
	if (likely(!v.deleted()) && !n_->has_changed(oldv, old_perm)) {
	found:
	    if (kp_ >= 0 && n_->value_is_stable_layer(kp_)) {
		root = n_->lv_[kp_].layer();
		if (root->has_split())
		    n_->lv_[kp_] = root = root->unsplit_ancestor();
		n_->unlock(v);
		ka_.shift();
		return root;
	    } else
		return 0;
	}
    }


    // Walk along leaves.
    while (1) {
	if (unlikely(v.deleted())) {
	    n_->unlock(v);
	    return root;
	}
	ki_ = leaf_type::bound_type::lower_with_position(ka_, *n_, kp_);
	if (kp_ >= 0) {
	    n_->lv_[kp_].prefetch(n_->keylenx_[kp_]);
	    goto found;
	} else if (likely(ki_ != n_->size() || !v.has_split(oldv))
		   || !(next = n_->safe_next())
		   || compare(ka_.ikey(), next->ikey_bound()) < 0)
	    goto found;
	n_->unlock(v);
	ti.mark(tc_leaf_retry);
	ti.mark(tc_leaf_walk);
	do {
	    n_ = next;
	    oldv = n_->stable();
	} while (!unlikely(oldv.deleted()) && (next = n_->safe_next())
		 && compare(ka_.ikey(), next->ikey_bound()) >= 0);
	n_->prefetch();
	v = n_->lock(oldv, ti.lock_fence(tc_leaf_lock));
    }
}

template <typename P>
inline node_base<P>* tcursor<P>::check_leaf_locked(node_type* root,
                                                   nodeversion_type v,
                                                   threadinfo& ti)
{
    if (node_type *next_root = get_leaf_locked(root, v, ti))
	return next_root;
    if (kp_ >= 0) {
	if (!n_->ksuf_equals(kp_, ka_))
	    kp_ = -1;
    } else if (ki_ == 0 && unlikely(n_->deleted_layer())) {
	n_->unlock();
	return reset_retry();
    }
    return 0;
}

template <typename P>
bool tcursor<P>::find_locked(threadinfo& ti)
{
    nodeversion_type v;
    node_type* root = root_;
    while (1) {
	n_ = root->reach_leaf(ka_, v, ti);
	root = check_leaf_locked(root, v, ti);
	if (!root) {
            state_ = kp_ >= 0;
	    return kp_ >= 0;
        }
    }
}

/*
  hyw:
	This is the cursor we use to build the 
	static tree

*/

template<typename P>
massnode<P>* unlocked_tcursor<P>::buildStatic(threadinfo& ti) {
  typedef typename P::ikey_type ikey_type;

  std::deque<leafvalue<P>> q;
  std::deque<uint8_t> keylenList;
  std::deque<ikey_type> keyList;
  std::deque<leafvalue<P>> link_or_value_list;
  std::vector<int> has_ksuf_list;
  std::vector<massnode<P>*> nodeList;
  std::deque<Str> ksufList;
  int kp = 0;
  int keylenx = 0;
  unsigned int massID = 1;
  int nkeys = 0;
  size_t ksufSize = 0;
  node_base<P>* root = const_cast <node_base<P>*> (root_);
  leaf<P> *next;

 nextMass:
  n_ = root -> leftmost();
 nextLeaf:
  n_ -> prefetch();
  perm_ = n_ -> permutation();
  nkeys += perm_.size();
  ksufSize += n_ -> ksuf_size();
  for (int i = 0; i < perm_.size(); i++) {
    kp = perm_[i];
    keyList.push_back(n_ -> ikey0_[kp]);
    keylenList.push_back(n_ -> keylenx_[kp]);
    link_or_value_list.push_back(n_ -> lv_[kp]);
    keylenx = n_ -> keylenx_[kp];
    if (n_ -> keylenx_is_layer(keylenx)) {
      q.push_back(n_ -> lv_[kp]);
    }
    if (n_ -> has_ksuf(kp)) {
      has_ksuf_list.push_back(1);
      ksufList.push_back(n_ -> ksuf(kp));
    }
    else
      has_ksuf_list.push_back(0);
  }
  if ((next = n_ -> safe_next())) {
    n_ = next;
    goto nextLeaf;
  }

  massnode<P>* newNode = massnode<P>::make(ksufSize, nkeys, ti);
  nodeList.push_back(newNode);
  for (int i = 0; i < nkeys; i++) {
    newNode -> keylenx_[i] = keylenList.front();
    keylenList.pop_front();
    newNode -> ikey0_[i] = keyList.front();
    keyList.pop_front();
    newNode -> lv_[i] = link_or_value_list.front();
    link_or_value_list.pop_front();
    if (leaf<P>::keylenx_is_layer(newNode -> keylenx_[i])) {
      newNode -> lv_[i].setX(massID);
      massID++;
    }
    if (has_ksuf_list[i]) {
      newNode -> ksuf_ -> assign(i, ksufList.front());
      ksufList.pop_front();
    }
  }

  keylenList.clear();
  keyList.clear();
  link_or_value_list.clear();
  has_ksuf_list.clear();
  ksufList.clear();

  if (q.size() != 0) {
    root = q.front().layer();
    q.pop_front();

    goto nextMass;
  }
  unsigned int id = 0;
  for (unsigned int i = 0; i < nodeList.size(); i++) {
    for (unsigned int j = 0; j < nodeList[i] -> nkeys_; j++) {
      if (leaf<P>::keylenx_is_layer(nodeList[i] -> keylenx_[j]))
	id = nodeList[i]->lv_[j].getX();
        (nodeList[i]) -> lv_[j] = nodeList[id];
    }
  }

  return nodeList[0];
}
} // namespace Masstree
#endif
