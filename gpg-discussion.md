# GnuPG (GPG) discussion

## About

GnuPG (GPG) is an encryption and validation utility based on Public Key Cryptography. It allows data to be encrypted in such a way that only specific people can decrypt it. It also allows people who receive data to verify that it came from the person who claimed to send it.

The Raspberry Pilot installation instructions include a step to let Gernby know when you are ready to receive a model file. Raspberry Pilot will not be able to issue steering commands without that file. Once Gernby knows your car is ready to receive steering commands, he will send the file to you by email. By utilizing GPG, he could publish one encrypted copy of the file while ensuring that only the intended recipients can use it. He will also be able to authorize new users to decrypt older versions of the model if needed for troubleshooting or A/B testing purposes.

Below are the steps required to generate your secret and public keys (called your "key pair") and for sharing your public key. This will simplify the model distribution while maintaining protections that are still required at this stage. This is only a temporary measure and the restriction will be lifted once more cars are validated.

## GPG Key Pair Generation

**Note: All of these steps are to be performed on your Pi, not any other workstation you may use to build and configure your Pi.**

These steps should be performed after the entire installation process has completed and the Pi is now sitting idle with all four key processes running. Therefore, this will be done after Gernby has emailed the current model file to you. This process is CPU intensive, so it should be left to run overnight on an idling Pi. If you interrupt the process, you will need to reissue the command and wait another 7 hours. This long process only needs to be executed once per install, but you will need to repeat it should you erase your SD card for any reason.

First, create a file in your home directory with the following contents, making substitutions as appropriate:

```
%echo Generating a default key  
Key-Type: default  
Subkey-Type: default  
Name-Real: <your Discord nickname>  
Name-Comment: <your car type>  
Name-Email: <your email>  
Expire-Date: 1000  
Passphrase: abc  
%commit  
%echo done  
```

Then at the command prompt run

`gpg --batch --generate-key <name of file with contents above>`

Wait 7 hours and validate that the command has completed successfully.

## Exporting and sharing your Public Key

Once you have generated your key pair, run the following command in your home directory.

`gpg --export -a`

Copy and paste the output into a PR to add the output to the end of the file named for your car type, such as `Honda Bosch`. You must include the break lines surrounding the key data. 

```
-----BEGIN PGP PUBLIC KEY BLOCK-----

mQINBE34z9wBEACT31iv9i8Jx/6MhywWmytSGWojS7aJwGiH/wlHQcjeleGnW8HF
Z8R73ICgvpcWM2mfx0R/YIzRIbbT+E2PJ+iTw0BTGU7irRKrdLXReH130K3bDg05
+DaYFf0qY/t/e4WDXRVnr8L28hRQ4/9SnvgNcUBzd0IDOUiicZvhkIm6TikL+xSr
5Gcn/PaJFS1VpbWklXaLfvci9l4fINL3vMyLiV/75b1laSP5LPEvbfd7W9T6HeCX
63epTHmGBmB4ycGqkwOgq6NxxaLHxRWlfylRXRWpI/9B66x8vOUd70jjjyqG+mhQ
+1+qfydeSW3R6Dr2vzDyDrBXbdVMTL2VFXqNG03FYcv191H7zJgPlJGyaO4IZxj+
+O8LaoJuFqAr8/+NX4K4UfWPvcrJ2i+eUkbkDJHo4GQK712/DtSLAA+YGeIF9HAn
zKvaMkZDMwY8z3gBSE/jMV2IcONvpUUOFPQgTmCvlJZAFTPeLTDv+HX8GfhmjAJY
<...>
3qp6L7jc6X3U+bn2m7u2cgEVbuAnSaKGoMSMnsd71Ltf1b6/DwvZz/HBttEgcgSm
PleHUVyBD4LDrcjTDK7zdEMw7b/cPBnu6CmTcogFEqvB4n9Yodo+4ij7AndUTz4J
j1p8vFlnHvhRg82MDfGUPJ+ujBjbYXROs+WAmaCQ8TgjZ3dAFNFrOqAbYu6QlY2x
fu7vj+ruc6ArdmBrOlsJFmNsxFRJfgdUug5JFIUN77GbjisHjWem8cY3szuyEke8
H2pi803CAuVtkaoNmNDHsEBieft34Zo0V+A/q2wkix3S9vyRjOKqhGrW30qxnV6Z
FexueWuO3qOQ0ZU5/TIH0kft2n45/RexeBq/Ip52zE1vEvTkQmBCfCGZmqTu+9Ro
8qsjecxVNxyVPlwhlimryiQ+dPaJYaOSfiwEEMh2MyV5c6t6qN9n6jFdiCLOlmmH
ZFA8xDodsofQEmlv+I/xyEZ7na6nxbpZVuPC3B0JFtY=
=sUYl
-----END PGP PUBLIC KEY BLOCK-----
```

Add a blank line before and after your key data for readability.
