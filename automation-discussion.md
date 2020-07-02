# Automating Tasks with a Mobile App

## Background

A big part of testing Raspberry Pilot is a process commonly referred to as "A/B testing". We are constantly being asked to report which of multiple settings and driving models are better or worse than others. It helps to be able to quickly and easily update settings in configuration files or replacing complete files and rebooting. Since the Raspberry Pi is running Linux, the image includes a running SSH server. SSH allows users to execute commands on remote hosts, using the ssh command as an authentication mechanism. By crafting specific command lines or small scripts, it becomes easy to make critical changes to your copy of Raspberry Pilot remotely. Finally, when you add an SSH app to your phone that supports remote command execution, now you can change settings or replace driving models in Raspberry Pilot just by pressing buttons on your phone screen.

See Raspberry SSH as one such example app on Android, available here: https://play.google.com/store/apps/details?id=uk.co.knowles_online.raspberryssh&hl=en_US There is a Lite version available, but only the paid version allows you to create enough automation buttons to meet our needs.

## Switching models

The first approach to automating tasks covered here includes specifying commands as part of the SSH session establishment. This is useful for small, simple tasks such as removing the current hdf5 files from ~/raspilot/models and copying a different one into the folder. I created a folder structure for holding each of the hdf5 files being tested under /home/ubuntu/buttons/model-*, with "model-*' meaning "model-1", "model-2", "model-3", and "model-4". I placed one hdf5 file in each final folder and will replace those files when new model bundles are released. Then I created buttons in the mobile app that erase the current model from ~/raspilot/models, copies a specific hdf5 file over to the models folder, and reboots the Raspberry Pi to make the change take effect. For example, this is the command behind the "Model 1" button in my Raspberry SSH mobile app:

`/bin/rm /home/ubuntu/raspilot/models/*5 && /bin/cp /home/ubuntu/buttons/model-1/* /home/ubuntu/raspilot/models && /usr/bin/sudo /sbin/reboot`

I have four Model buttons, each one corresponding to loading an hdf5 file sitting in a specific directory before rebooting the Pi to make it take effect. When new models are published, I'll copy the new scalers into the models folder and place up to four hdf5 files in each of the model folders associated with the buttons.

## Changing values in kegman.json

The second approach uses the open source configuration management tool called Ansible. Ansible allows system administrators to use a standardized language when specifying which states or conditions need to be met on a given system. In this example, I am going to use Ansible to delete the dampTime parameter from the kegman.json file and write a new one in with a specific value. 

Using the same folder structure created for the models, I created /home/ubuntu/buttons/playbooks. Inside that folder are numerous yaml files -- each one corresponds to setting a specific parameter in kegman.json to a specific value. No reboot is required when making these changes. Each playbook first deletes the line with the specified parameter being updated and then adds a new version of that line with the parameter set to the desired value. For example, the playbook that sets "dampTime" to "0.05" would be this:

```
---
- hosts: localhost
  tasks:

  - name: Remove dampTime
    lineinfile:
      path: /home/ubuntu/kegman.json
      regexp: 'dampTime*.:'
      state: absent

  - name: Set dampTime to 0.05
    lineinfile:
      path: /home/ubuntu/kegman.json
      insertafter: dampMPC
      line: '  "dampTime": "0.05",'
```

The command behind the button in the mobile app to set this value is:

`/usr/bin/ansible-playbook /home/ubuntu/buttons/playbooks/dampTime-0.05.yaml`

You will end up with several buttons that will set specific parameters in kegman.json to whatever value is specified in the playbook associated with each button.

## Future goals

In the future, the goal is to learn how to change the kegman settings by a relative amount while restricting the values to stay within safe limits. For now, only specific values for specific parameters are supported.
