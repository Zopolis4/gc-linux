#ifndef __GCGX__
#define __GCGX__

int gcngx_mmap(struct fb_info *info, struct vm_area_struct *vma);
int gcngx_ioctl(struct fb_info *info, unsigned int cmd,unsigned long arg);

int gcngx_init(struct fb_info *info);
void gcngx_exit(struct fb_info *info);

void gcngx_vtrace(void);

#endif
