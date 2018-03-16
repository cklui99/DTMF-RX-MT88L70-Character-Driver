/**
 * @file   dtmf_rx.c
 * @author CK Lui
 * @date   Jan 9, 2018
 * @description
 * A kernel module for controlling a dtmf receiver chip to read the dtmf data from gpios.
 * It has full support for interrupts and for sysfs entries so that an interface
 * can be created to the inputa signal std and its related data, and that can be configured from Linux userspace.
 * The sysfs entry appears at /sys/dtmf/gpio73
 * REFERENCES: http://www.derekmolloy.ie/
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/time.h>       // Using the clock to measure time between button presses
#define  DEBOUNCE_TIME 20    ///< The default bounce time -- 20ms
MODULE_LICENSE("GPL");
MODULE_AUTHOR("CK Lui");
MODULE_DESCRIPTION("A Linux character device driver for DTMF Receiver MT88L70 implemented in Beaglebone with accessible outputs in file system.");
MODULE_VERSION("0.1");
static bool isRising = 1;                   ///< Rising edge is the default IRQ property
module_param(isRising, bool, S_IRUGO);      ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(isRising, " Rising edge = 1 (default), Falling edge = 0");  ///< parameter description
static unsigned int gpioDTMFdetected = 73;       ///< Default GPIO is 73
module_param(gpioDTMFdetected, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioDTMFdetected, " GPIO assigned to DTMFdetected signal (default=73)");  ///< parameter description
static unsigned int gpioDTMFdata1 = 86;       ///< Default GPIO is 86
module_param(gpioDTMFdata1, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioDTMFdata1, " GPIO assigned to DTMFdata1 signal (default=86)");  ///< parameter description
static unsigned int gpioDTMFdata2 = 75;       ///< Default GPIO is 75
module_param(gpioDTMFdata2, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioDTMFdata2, " GPIO assigned to DTMFdata2 signal (default=75)");  ///< parameter description
static unsigned int gpioDTMFdata3 = 76;       ///< Default GPIO is 76
module_param(gpioDTMFdata3, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioDTMFdata3, " GPIO assigned to DTMFdata3 signal (default=76)");  ///< parameter description
static unsigned int gpioDTMFdata4 = 77;       ///< Default GPIO is 77
module_param(gpioDTMFdata4, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioDTMFdata4, " GPIO assigned to DTMFdata4 signal (default=77)");  ///< parameter description
static unsigned int gpioDTMFpd = 87;           ///< Default GPIO is 87
module_param(gpioDTMFpd, uint, S_IRUGO);       ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioDTMFpd, " GPIO assigned to DTMFpd signal (default=87)");         ///< parameter description
//DTMF detected indicator @ gpio51
static unsigned int gpioLED = 51;           ///< Default GPIO is 51
module_param(gpioLED, uint, S_IRUGO);       ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioLED, " GPIO LED number (default=51)");         ///< parameter description

 static char   gpioName[8] = "gpioXXX";      ///< Null terminated default string -- just in case
static int    irqNumber;                    ///< Used to share the IRQ number within this file
static int    numberPresses = 0;            ///< For information, store the number of button presses
static bool   DTMFdetected = 0;                 ///< Use to store DTMFdetected (off by default)
static unsigned int   DTMFdata1 = 0;                    ///< Use to store data 1 (0 by default)
static unsigned int  DTMFdata2 = 0;                    ///< Use to store data 2 (0 by default)
static  unsigned int   DTMFdata3 = 0;                    ///< Use to store data 3 (0 by default)
static unsigned int   DTMFdata4 = 0;                    ///< Use to store data 4 (0 by default)
static unsigned int   DTMFdigit = 0;                    ///< Use to store dtmf digit recieved (0 by default)
static int *digit=0;                    ///< Use to store digit received (0 by default)
static bool   isDebounce = 1;               ///< Use to store the debounce state (on by default)
static bool   isDTMFpd = 0;               ///< Use to store the DTMFpd state (off by default)
static bool     ledOn = 0;          ///< Use to show dtmf detected status (off by default)
static struct timespec ts_last, ts_current, ts_diff;  ///< timespecs from linux/time.h (has nano precision)

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  dtmfrx_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

/** @brief A callback function to output the numberPresses variable
 *  @param kobj represents a kernel object device that appears in the sysfs filesystem
 *  @param attr the pointer to the kobj_attribute struct
 *  @param buf the buffer to which to write the number of presses
 *  @return return the total number of characters written to the buffer (excluding null)
 */
static ssize_t numberPresses_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", numberPresses);
}

/** @brief A callback function to read in the numberPresses variable
 *  @param kobj represents a kernel object device that appears in the sysfs filesystem
 *  @param attr the pointer to the kobj_attribute struct
 *  @param buf the buffer from which to read the number of presses (e.g., reset to 0).
 *  @param count the number characters in the buffer
 *  @return return should return the total number of characters used from the buffer
 */
static ssize_t numberPresses_store(struct kobject *kobj, struct kobj_attribute *attr,
                                   const char *buf, size_t count){
   sscanf(buf, "%du", &numberPresses);
   return count;
}

/** @Displays if the LED is on or off */
static ssize_t ledOn_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", ledOn);
}

/** @Displays DTMFdetetced */
static ssize_t DTMFdetected_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", DTMFdetected);
}

/** @Displays DTMFdata1*/
static ssize_t DTMFdata1_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", DTMFdata1);
}
 /** @Displays DTMFdata2*/
static ssize_t DTMFdata2_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", DTMFdata2);
}
/** @Displays DTMFdata3*/
static ssize_t DTMFdata3_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", DTMFdata3);
}
/** @Displays DTMFdata4*/
static ssize_t DTMFdata4_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", DTMFdata4);
}

/** @Displays if DTMFpd is on or off */
static ssize_t isDTMFpd_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", isDTMFpd);
}
/** @brief Stores and sets the debounce state */
static ssize_t isDTMFpd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   unsigned int temp;
   sscanf(buf, "%du", &temp);                // use a temp varable for correct int->bool
   gpio_set_value(gpioDTMFpd,0);
   isDTMFpd = temp;
   if(isDTMFpd) { gpio_set_value(gpioDTMFpd, isDTMFpd);
      printk(KERN_INFO "gpioDTMFpd: DTMFpd is set \n");
   }
   else { gpio_set_debounce(gpioDTMFpd, 0);  // set the debounce time to 0
      printk(KERN_INFO "gpioDTMFdetetced: Debounce off\n");
   }
   return count;
}

/** @brief Displays the last time the button was pressed -- manually output the date (no localization) */
static ssize_t lastTime_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%.2lu:%.2lu:%.2lu:%.9lu \n", (ts_last.tv_sec/3600)%24,
          (ts_last.tv_sec/60) % 60, ts_last.tv_sec % 60, ts_last.tv_nsec );
}

/** @brief Display the time difference in the form secs.nanosecs to 9 places */
static ssize_t diffTime_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%lu.%.9lu\n", ts_diff.tv_sec, ts_diff.tv_nsec);
}

/** @brief Displays if button debouncing is on or off */
static ssize_t isDebounce_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", isDebounce);
}
/** @brief Stores and sets the debounce state */
static ssize_t isDebounce_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   unsigned int temp;
   sscanf(buf, "%du", &temp);                // use a temp varable for correct int->bool
   gpio_set_debounce(gpioDTMFdetected,0);
   isDebounce = temp;
   if(isDebounce) { gpio_set_debounce(gpioDTMFdetected, DEBOUNCE_TIME);
      printk(KERN_INFO "gpioDTMFdetetced: Debounce on\n");
   }
   else { gpio_set_debounce(gpioDTMFdetected, 0);  // set the debounce time to 0
      printk(KERN_INFO "gpioDTMFdetetced: Debounce off\n");
   }
   return count;
}
/**  Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The count variable is associated with the numberPresses variable and it is to be exposed
 *  with mode 0660 using the numberPresses_show and numberPresses_store functions above
 */
static struct kobj_attribute count_attr = __ATTR(numberPresses, 0660, numberPresses_show, numberPresses_store);
static struct kobj_attribute debounce_attr = __ATTR(isDebounce, 0660, isDebounce_show, isDebounce_store);
static struct kobj_attribute DTMFpd_attr = __ATTR(isDTMFpd, 0660, isDTMFpd_show, isDTMFpd_store);


/**  The __ATTR_RO macro defines a read-only attribute. There is no need to identify that the
 *  function is called _show, but it must be present. __ATTR_WO can be  used for a write-only
 *  attribute but only in Linux 3.11.x on.
 */

 static struct kobj_attribute DTMFdetected_attr  = __ATTR_RO(DTMFdetected);  ///< the DTMFdetected kobject attr
static struct kobj_attribute ledOn_attr = __ATTR_RO(ledOn);     ///< the DTMFpd  kobject attr
static struct kobj_attribute time_attr  = __ATTR_RO(lastTime);  ///< the last time pressed kobject attr
static struct kobj_attribute diff_attr  = __ATTR_RO(diffTime);  ///< the difference in time attr
 static struct kobj_attribute DTMFdata1_attr  = __ATTR_RO(DTMFdata1);  ///< the DTMFdata1 kobject attr
static struct kobj_attribute DTMFdata2_attr  = __ATTR_RO(DTMFdata2);  ///< the DTMFdata2 kobject attr
static struct kobj_attribute DTMFdata3_attr  = __ATTR_RO(DTMFdata3);  ///< the DTMFdata3 kobject attr
static struct kobj_attribute DTMFdata4_attr  = __ATTR_RO(DTMFdata4);  ///< the DTMFdata4 kobject attr
/**  The dtmf_attrs[] is an array of attributes that is used to create the attribute group below.
 *  The attr property of the kobj_attribute is used to extract the attribute struct
 */
static struct attribute *dtmf_attrs[] = {
      &count_attr.attr,                  ///< The number of button presses
      &DTMFpd_attr.attr,                  ///< Is the DTMFpd state true or false
      &DTMFdetected_attr.attr,                  ///< Is the DTMF detected gpio state true or false
      &ledOn_attr.attr,                  ///< Is the LED on or off?
      &time_attr.attr,                   ///< Time of the last button press in HH:MM:SS:NNNNNNNNN
      &diff_attr.attr,                   ///< The difference in time between the last two presses
      &debounce_attr.attr,               ///< Is the debounce state true or false
      &DTMFdata1_attr.attr,               ///< the DTMFdata1
      &DTMFdata2_attr.attr,               ///< the DTMFdata2
      &DTMFdata3_attr.attr,               ///< the DTMFdata3
      &DTMFdata4_attr.attr,               ///< the DTMFdata4
      NULL,
};

/**  The attribute group uses the attribute array and a name, which is exposed on sysfs -- in this
 *  case it is gpio73, which is automatically defined in the dtmfrx_init() function below
 *  using the custom kernel parameter that can be passed when the module is loaded.
 */
static struct attribute_group attr_group = {
      .name  = gpioName,                 ///< The name is generated in ebbButton_init()
      .attrs = dtmf_attrs,                ///< The attributes array defined just above
};

static struct kobject *dtmfrx_kobj;

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init dtmfrx_init(void){
   int result = 0;
   unsigned long IRQflags = IRQF_TRIGGER_RISING;      // The default is a rising-edge interrupt

   printk(KERN_INFO "DTMF DETECTED: Initializing the DTMF DETECTED @TOE LKM\n");
   sprintf(gpioName, "gpio%d", gpioDTMFdetected);           // Create the gpio73 name for /sys/dtmf/gpio73

   // create the kobject sysfs entry at /sys/dtmf
   dtmfrx_kobj = kobject_create_and_add("dtmf", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
   if(!dtmfrx_kobj){
      printk(KERN_ALERT "DTMF DETECTED: failed to create kobject mapping\n");
      return -ENOMEM;
   }
   // add the attributes to /sys/dtmf/ -- for example, /sys/dtmf/gpio73/numberPresses
   result = sysfs_create_group(dtmfrx_kobj, &attr_group);
   if(result) {
      printk(KERN_ALERT "DTMF DETECTED: failed to create sysfs group\n");
      kobject_put(dtmfrx_kobj);                          // clean up -- remove the kobject sysfs entry
      return result;
   }
   getnstimeofday(&ts_last);                          // set the last time to be the current time
   ts_diff = timespec_sub(ts_last, ts_last);          // set the initial time difference to be 0

   // Going to set up the LED. It is a GPIO in output mode and will be off by default
   ledOn = false;
   gpio_request(gpioLED, "sysfs");          // gpioLED is hardcoded to51, request it
   gpio_direction_output(gpioLED, ledOn);   // Set the gpio to be in output mode and off
// gpio_set_value(gpioLED, ledOn);          // Not required as set by line above (here for reference)
   gpio_export(gpioLED, false);             // Causes gpio51 to appear in /sys/class/gpio
                                            // the bool argument prevents the direction from being changed
   gpio_request(gpioDTMFdetected, "sysfs");       // Set up the gpioDTMFdetected
  gpio_direction_input(gpioDTMFdetected);        // Set the DTMF detected GPIO to be an input
  gpio_set_debounce(gpioDTMFdetected, DEBOUNCE_TIME); // Debounce the DTMF detected GPIO with a delay of 200ms
   gpio_export(gpioDTMFdetected, false);          // Causes gpio73 to appear in /sys/class/gpio
                                            // the bool argument prevents the direction from being changed
   gpio_request(gpioDTMFdata1, "sysfs");       // Set up the gpioDTMFdata1
  gpio_direction_input(gpioDTMFdata1);        // Set the DTMF Data 1 GPIO to be an input
  gpio_set_debounce(gpioDTMFdata1, DEBOUNCE_TIME); // Debounce the DTMF detected GPIO with a delay of 200ms
   gpio_export(gpioDTMFdata1, false);          // Causes gpio86 to appear in /sys/class/gpio
                                            // the bool argument prevents the direction from being changed
   gpio_request(gpioDTMFdata2, "sysfs");       // Set up the gpioDTMFdata2
  gpio_direction_input(gpioDTMFdata2);        // Set the DTMF Data 2 GPIO to be an input
  gpio_set_debounce(gpioDTMFdata2, DEBOUNCE_TIME); // Debounce the DTMF detected GPIO with a delay of 200ms
   gpio_export(gpioDTMFdata2, false);          // Causes gpio75 to appear in /sys/class/gpio
                                            // the bool argument prevents the direction from being changed
   // Perform a quick test to see that the button is working as expected on LKM load
   printk(KERN_INFO "DTMF DETECTED: The DTMF detected GPIO state is currently: %d\n", gpio_get_value(gpioDTMFdetected));

   /// GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   irqNumber = gpio_to_irq(gpioDTMFdetected);
   printk(KERN_INFO "DTMF DETECTED: The button is mapped to IRQ: %d\n", irqNumber);

   if(!isRising){                           // If the kernel parameter isRising=0 is supplied
      IRQflags = IRQF_TRIGGER_FALLING;      // Set the interrupt to be on the falling edge
   }
   // This next call requests an interrupt line
   result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) dtmfrx_irq_handler, // The pointer to the handler function below
                        IRQflags,              // Use the custom kernel param to set interrupt type
                        "dtmfdetected_handler",  // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay
   return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit dtmfrx_exit(void){
   printk(KERN_INFO "DTMF DETECTED: The dtmf was detected %d times\n", numberPresses);
  ledOn = false;
   kobject_put(dtmfrx_kobj);                   // clean up -- remove the kobject sysfs entry
   gpio_set_value(gpioLED, ledOn);              // Turn the LED off, makes it clear the device was unloaded
   gpio_unexport(gpioLED);                  // Unexport the LED GPIO
   free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   gpio_unexport(gpioDTMFdetected);               // Unexport the DTMF detected GPIO
   gpio_free(gpioLED);                      // Free the LED GPIO
   gpio_free(gpioDTMFdetected);                   // Free the DTMF detected GPIO
   printk(KERN_INFO "DTMF DETECTED: Goodbye from the DTMF DETECTED LKM!\n");
}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t dtmfrx_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
   DTMFdata1 = gpio_get_value(gpioDTMFdata1);       ///< DTMF Data 1 received
   DTMFdata2 = gpio_get_value(gpioDTMFdata2);       ///< DTMF Data 2 received
   DTMFdata3 = gpio_get_value(gpioDTMFdata3);       ///< DTMF Data 3 received
   DTMFdata4 = gpio_get_value(gpioDTMFdata4);       ///< DTMF Data 4 received
   DTMFdigit = (DTMFdata1 << 1) & (DTMFdata1 << 1) & (DTMFdata3 << 3) & (DTMFdata4 << 4);       ///< DTMF Digit received
   ledOn = true;                      // Invert the LED state on each button press
   gpio_set_value(gpioLED, ledOn);      // Set the physical LED accordingly
   getnstimeofday(&ts_current);         // Get the current time as ts_current
   ts_diff = timespec_sub(ts_current, ts_last);   // Determine the time difference between last 2 presses
   ts_last = ts_current;                // Store the current time as the last time ts_last
   printk(KERN_INFO "DTMF DETECTED: The DTMF detected GPIO state is currently: %d\n", gpio_get_value(gpioDTMFdetected));
   numberPresses++;                     // Global counter, will be outputted when the module is unloaded
switch(DTMFdigit) {
   case 1  :
      *digit=1;
      break;
   case 2  :
      *digit=2;
      break;
   case 3  :
      *digit=3;
      break;
   case 4  :
      *digit=4;
      break;
   case 5  :
      *digit=5;
      break;
   case 6  :
      *digit=6;
      break;
   case 7  :
      *digit=7;
      break;
   case 8  :
      *digit=8;
      break;
   case 9  :
      *digit=9;
      break;
   case 10  :
      *digit=10;
      break;
   case 11  :
      *digit=11; //"*"
      break;
   case 12  :
      *digit=12; //"#"
      break;
   case 13  :
      *digit=13; //"A"
      break;
   case 14  :
      *digit=14; //"B"
      break;
   case 15  :
      *digit=15; //"C"
      break;
   case 16  :
      *digit=16; //"D"
      break;
   default :
      *digit=99; //invalid
}
  printk(KERN_INFO "DTMF DIGIT DETECTED: The DTMF digit is : %x\n",  *digit);
   return (irq_handler_t) IRQ_HANDLED;  // Announce that the IRQ has been handled correctly
}

// This next calls are  mandatory -- they identify the initialization function
// and the cleanup function (as above).
module_init(dtmfrx_init);
module_exit(dtmfrx_exit);
