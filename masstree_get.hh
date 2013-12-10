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
     binary search for scursor in a massnode
*/
template <typename P>
inline int scursor<P>::lower_bound_binary() const
{
    int l = 0, r = numKeys_;
    while (l < r) {
        int m = (l + r) >> 1;
        int cmp = key_compare(ka_, *n_, m);
        if (cmp < 0)
            r = m;
        else if (cmp == 0)
            return m;
        else
            l = m + 1;
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

  ti.totalAllocSize = 0;
  ti.ksufSize = 0;
  ti.totalNumkeys = 0;
  ti.totalMassnode = 0;

 nextMass:
  n_ = root -> leftmost();
 nextLeaf:
  n_ -> prefetch();
  perm_ = n_ -> permutation();
  nkeys += perm_.size();
  //ksufSize += n_ -> ksuf_size();
  for (int i = 0; i < perm_.size(); i++) {
    kp = perm_[i];
    keyList.push_back(n_ -> ikey0_[kp]);
    keylenList.push_back(n_ -> keylenx_[kp]);
    link_or_value_list.push_back(n_ -> lv_[kp]);
    keylenx = n_ -> keylenx_[kp];
    //std::cout << "keylenx_[" << kp << "] = " << int(n_ -> keylenx_[kp]) << "\n";
    //std::cout << "ikey0_[" << kp << "] = " << n_ -> ikey0_[kp] << "\n";
    if (n_ -> keylenx_is_layer(keylenx)) {
      q.push_back(n_ -> lv_[kp]);
    }

    if (n_ -> has_ksuf(kp)) {
      has_ksuf_list.push_back(1);
      ksufSize += n_ -> ksuf(kp).len;
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
  //std::cout << "nkeys after node creation = " << nkeys << "\n";
  //std::cout << "nodeList size = " << nodeList.size() << "\n";
  nodeList.push_back(newNode);
  //std::cout << "nkeys = " << nkeys << "\n";
  char* curpos = newNode -> get_ksuf();
  char* startpos = newNode -> get_ksuf();
  for (int i = 0; i < nkeys; i++) {
    //std::cout << "count = " << i << "\tnkeys = " << nkeys << "\n";
    if (keylenList.empty())
      std::cout << "keylenList Empty!\n";
    newNode -> get_keylenx()[i] = keylenList.front();
    //std::cout << "keylenx_[" << i << "] = " << keylenList.front() << "\n";
    keylenList.pop_front();
    if (keyList.empty())
      std::cout << "keyList Empty!\n";
    newNode -> get_ikey0()[i] = keyList.front();
    //std::cout << "ikey0_[" << i << "] = " << keyList.front() << "\n";
    keyList.pop_front();

    if (link_or_value_list.empty())
      std::cout << "link_or_value_list Empty!\n";
    newNode -> get_lv()[i] = link_or_value_list.front();
    link_or_value_list.pop_front();

    if (leaf<P>::keylenx_is_layer(newNode -> get_keylenx()[i])) {
      newNode -> get_lv()[i].setX(massID);
      massID++;
    }

    if (has_ksuf_list[i]) {
      newNode -> get_ksuf_pos_offset()[i] = (uint32_t)(curpos - startpos);
      memcpy(curpos, ksufList.front().s, ksufList.front().len);
      curpos += ksufList.front().len;
      ksufList.pop_front();
    }
    else
      newNode -> get_ksuf_pos_offset()[i] = (uint32_t)(curpos - startpos);
  }

  newNode -> get_ksuf_pos_offset()[nkeys] = (uint32_t)(curpos - startpos);
  /*  
  for (int i = 0; i < nkeys; i++) {
    //if (has_ksuf_list[i])
    //std::cout << "ksuf_[" << i << "] = " <<newNode -> ksuf(i) << "\n";
  }
  */

  //for (int i = 0; i < link_or_value_list.size(); i++) {
  //std::cout << "link_or_value_list_[" << i << "] = " << link_or_value_list[i] << "\n";
  //}
  
  keylenList.clear();
  keyList.clear();
  link_or_value_list.clear();
  has_ksuf_list.clear();
  ksufList.clear();
  nkeys = 0;
  ksufSize = 0;

  if (q.size() != 0) {
    root = q.front().layer();
    q.pop_front();

    goto nextMass;
  }

  unsigned int id = 0;
  for (unsigned int i = 0; i < nodeList.size(); i++) {
    for (unsigned int j = 0; j < nodeList[i] -> nkeys_; j++) {
      if (leaf<P>::keylenx_is_layer(nodeList[i] -> get_keylenx()[j])) {
        id = nodeList[i] -> get_lv()[j].getX();
        (nodeList[i]) -> get_lv()[j] = nodeList[id];
      }
    }
  }

  std::cout << "buildStatic finish\n";
  return nodeList[0];
}

/*
    hyw:
      static massnode find 
*/
template <typename P>
bool scursor<P>::find()
{
    bool ksuf_match = false;
    int kp, keylenx = 0;
    int count = 1;
    n_ = static_cast<massnode<P>*>(root_);

nextNode:
    n_->prefetch();
    numKeys_ = n_->nkeys_;
    //std::cout << "\t" << count << ". # keys " << numKeys_ << "\n";
    count++;
    kp = lower_bound_binary();
    if (kp >= 0) {
      keylenx = n_ -> get_keylenx()[kp];
      // TODO: I am not sure if we still need this fence since we are static
      // fence();
      lv_ = n_ -> get_lv()[kp];
      lv_.prefetch(keylenx);
      ksuf_match = n_->ksuf_equals(kp, ka_, keylenx);
    }
    if (kp < 0) {
      return false;
    }
    else if (n_->keylenx_is_layer(keylenx)) {
      ka_.shift();
      n_ = static_cast<massnode<P>*>(lv_.layer());
      goto nextNode;
    } 
    else
      return ksuf_match;
}

/*
    hyw:
      static massnode find 
*/
template <typename P>
bool scursor<P>::scan()
{
    bool ksuf_match = false;
    int kp, keylenx = 0;
    int count = 1;
    n_ = static_cast<massnode<P>*>(root_);

    if (!nodeTrace_.empty())
      return false;

 nextNode:
    n_->prefetch();
    numKeys_ = n_->nkeys_;
    std::cout << "\t" << count << ". # keys " << numKeys_ << "\n";
    count++;
    kp = lower_bound_binary();
    if (kp >= 0) {
      keylenx = n_ -> get_keylenx()[kp];
      // TODO: I am not sure if we still need this fence since we are static
      // fence();
      lv_ = n_ -> get_lv()[kp];
      lv_.prefetch(keylenx);
      ksuf_match = n_->ksuf_equals(kp, ka_, keylenx);
    }
    if (kp < 0) {
      return false;
    }
    else if (n_->keylenx_is_layer(keylenx)) {
      ka_.shift();
      nodeTrace_.push(n_);
      posTrace_.push(kp);
      n_ = static_cast<massnode<P>*>(lv_.layer());
      goto nextNode;
    }

    if (!ksuf_match)
      return false;

    pos_ = kp;
    count = 0;
 nextKey:
    //std::cout << "\t\t" << ". # keys " << numKeys_ << "\n";
    for (int i = pos_; i < (int)numKeys_; i++) {
      keylenx = n_ -> get_keylenx()[i];
      if (n_ -> keylenx_is_layer(keylenx)) {
        nodeTrace_.push(n_);
        posTrace_.push(i);
        lv_ = n_ -> get_lv()[i];
        n_ = static_cast<massnode<P>*>(lv_.layer());
        pos_ = 0;
        if (n_)
          numKeys_ = n_ -> nkeys_;
        else {
          std::cout << "Error, node empty!!!\n";
          return false;
        }
        goto nextKey;
      }
      else {
        //keyList_.push_back(curNode -> get_ikey0()[i]);
        valueList_.push_back(n_ -> get_lv()[i]);
        //std::cout << n_ -> get_ikey0()[i] << "\n";
        count++;
        if (count >= range_)
          goto finish;
      }
    }
    if (nodeTrace_.empty()) {
      std::cout << "Error, nodeTrace empty!\n";
      return false;
    }
    else {
      n_ = static_cast<massnode<P>*>(nodeTrace_.top());
      pos_ = posTrace_.top() + 1;
      numKeys_ = n_ -> nkeys_;
      nodeTrace_.pop();
      posTrace_.pop();
      goto nextKey;
    }

 finish:
    return true;
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

} // namespace Masstree
#endif
